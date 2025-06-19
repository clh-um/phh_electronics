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
const int relay1 = 14;
const int relay2 = 12;
const int relay3 = 26;
const int buzzer = 27;

#define DHTPIN 4
#define SEALEVELPRESSURE_HPA (1013.25)
#define DHTTYPE DHT22

// Configuration constants
struct Config {
  static constexpr float TEMP_THRESHOLD1 = 34.00;  // Changed from 34.50 to 34.00
  static constexpr float TEMP_THRESHOLD2 = 33.00;
  static constexpr unsigned long WATER_TIMEOUT = 10UL * 60UL * 1000UL;
  static constexpr unsigned long LCD_INTERVAL = 2000;
  static constexpr unsigned long SENSOR_INTERVAL = 2000;
  static constexpr unsigned long FAST_SENSOR_INTERVAL = 500;
  static constexpr unsigned long RETRY_INTERVAL = 100;
  static constexpr unsigned long STARTUP_GRACE_PERIOD = 30000; // 30 seconds grace period
  static constexpr unsigned long HEATER1_SHUTDOWN_DELAY = 2UL * 60UL * 1000UL; // 2 minutes
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
bool previousState3 = false, currentState3 = false; // Added for relay2
unsigned long lastWaterSensorCheck = 0;
const unsigned long WATER_SENSOR_CHECK_INTERVAL = 100; // Check every 100ms
phh::device::DeviceID deviceId; // Keep the original DeviceID class
bool lastAirFlowStatus = false;
unsigned long airFlowLostStartTime = 0;
const unsigned long AIR_FLOW_LOST_CONFIRMATION_TIME = 3000;

// NEW: Two-stage heater control variables
bool heater1Active = false;  // State of heater 1 (relay1)
bool heater2Active = false;  // State of heater 2 (relay2)
bool heater3Active = false;  // State of heater 3 (relay3)
unsigned long heater2ShutdownTime = 0;  // When heater 2 was shut down due to high temp
bool heater2ShutdownDueToTemp = false;  // Flag to track if heater 2 was shut down due to temperature
unsigned long highTempStartTime = 0;    // When thermocouple 1 first exceeded threshold
bool highTempTimerActive = false;       // Whether we're tracking high temperature duration

// NEW: Airflow monitoring with single sensor
float previousPressure = 0.0;
unsigned long lastPressureTime = 0;
const unsigned long PRESSURE_SAMPLE_INTERVAL = 500; // 0.5 second in milliseconds

// NEW: Relay state tracking for sensor pause
bool lastRelay1State = false;
bool lastRelay2State = false;
bool lastRelay3State = false;
unsigned long relayChangeTime = 0;
const unsigned long SENSOR_PAUSE_DURATION = 1500; // 1.5 seconds in milliseconds
bool sensorsPaused = false;

// NEW: System initialization tracking
unsigned long systemStartTime = 0;
bool systemFullyInitialized = false;

// Track override states for each relay
bool relays_override = false;
bool relay_override_states[3] = {false, false, false};

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
  
  // Initialize relay states
  lastRelay1State = digitalRead(relay1);
  lastRelay2State = digitalRead(relay2);
  lastRelay3State = digitalRead(relay3);
  
  Serial.println("-- Smart Prefilled Humidifier Heater System Ready --");
  Serial.println("-- Starting 30-second initialization grace period --");
}

void initializeHardware() {
  lcd.init();
  lcd.backlight();
  
  pinMode(LiqSensorPin, INPUT);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  
  // Ensure all relays start OFF
  digitalWrite(relay1, LOW);
  digitalWrite(relay2, LOW);
  digitalWrite(relay3, LOW);
  
  // Initialize heater states
  heater1Active = false;
  heater2Active = false;
  heater3Active = false;
  
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

// NEW: Function to check for relay state changes
void checkRelayStateChanges() {
  bool currentRelay1State = digitalRead(relay1);
  bool currentRelay2State = digitalRead(relay2);
  bool currentRelay3State = digitalRead(relay3);
  
  // Check if any relay state has changed
  if (currentRelay1State != lastRelay1State || 
      currentRelay2State != lastRelay2State || 
      currentRelay3State != lastRelay3State) {
    
    // Record the time of relay change
    relayChangeTime = millis();
    sensorsPaused = true;
    
    Serial.println("Relay state change detected - pausing sensor readings for 1.5 seconds");
    
    // Update last known states
    lastRelay1State = currentRelay1State;
    lastRelay2State = currentRelay2State;
    lastRelay3State = currentRelay3State;
  }
  
  // Check if pause duration has elapsed
  if (sensorsPaused && (millis() - relayChangeTime >= SENSOR_PAUSE_DURATION)) {
    sensorsPaused = false;
    Serial.println("Sensor readings resumed after relay switching pause");
  }
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
  } else if (!airFlowStatus) {
    currentSystemState = AIRFLOW_WARNING;
  } else if (!waterDetected) {
    currentSystemState = WATER_WARNING;
  } else {
    currentSystemState = NORMAL_OPERATION;
  }
}

void handleWaterSensorUpdate() {
  if (waterDetectedFlag) {
    waterDetectedFlag = false;
    waterDetected = digitalRead(LiqSensorPin) == LOW;
      
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
    heater1Active = false;
    heater2Active = false;
    heater3Active = false;
    currentState1 = false;
    currentState2 = false;
    currentState3 = false;
    return;
  }

  if (relays_override) {
      // Enforce all relay override states
      digitalWrite(relay1, relay_override_states[0] ? HIGH : LOW);
      digitalWrite(relay2, relay_override_states[1] ? HIGH : LOW);
      digitalWrite(relay3, relay_override_states[2] ? HIGH : LOW);
      heater1Active = relay_override_states[0];
      heater2Active = relay_override_states[1];
      heater3Active = relay_override_states[2];
      // currentState1, currentState2, currentState3 as needed
      return; // Skip automatic logic
  }

  // During initialization, keep heaters off for safety
  if (currentSystemState == INITIALIZING) {
    digitalWrite(relay1, LOW);
    digitalWrite(relay2, LOW);
    digitalWrite(relay3, LOW);
    heater1Active = false;
    heater2Active = false;
    heater3Active = false;
    currentState1 = false;
    currentState2 = false;
    currentState3 = false;
    return;
  }

  // Get sensor readings
  float Tt1 = getThermocouple1();
  float Tt2 = getThermocouple2();

  if (isnan(Tt1) || Tt1 < -50 || Tt1 > 150) {
    Tt1 = -999.0;  // Error indicator
  }
  if (isnan(Tt2) || Tt2 < -50 || Tt2 > 150) {
    Tt2 = -999.0;  // Error indicator
  }
  
  // NEW: Two-stage temperature control for heaters 1 and 2
  handleTwoStageHeaterControl(Tt1);
  
  // Control heater 3 based on thermocouple 2 (unchanged logic)
  if (Tt2 > Config::TEMP_THRESHOLD2 || Tt2 < 0) {
    digitalWrite(relay3, LOW);
    heater3Active = false;
    currentState2 = false;
  } else {
    digitalWrite(relay3, HIGH);
    heater3Active = true;
    currentState2 = true;
  }
}

void handleTwoStageHeaterControl(float Tt1) {
  unsigned long currentTime = millis();
  
  // Check if temperature is above threshold
  if (Tt1 > Config::TEMP_THRESHOLD1 && Tt1 > 0) {
    
    // Start tracking high temperature time if not already tracking
    if (!highTempTimerActive) {
      highTempStartTime = currentTime;
      highTempTimerActive = true;
      Serial.println("High temperature detected - starting timer");
    }
    
    // Stage 1: Immediately shut down heater 2 when temp > 34°C
    if (heater2Active) {
      digitalWrite(relay2, LOW);
      heater2Active = false;
      heater2ShutdownDueToTemp = true;
      heater2ShutdownTime = currentTime;
      Serial.println("Stage 1: Heater 2 shut down due to high temperature");
    }
    
    // Stage 2: After 2 minutes of continuous high temperature, shut down heater 1
    unsigned long highTempDuration = currentTime - highTempStartTime;
    if (highTempDuration >= Config::HEATER1_SHUTDOWN_DELAY) {
      if (heater1Active) {
        digitalWrite(relay1, LOW);
        heater1Active = false;
        Serial.println("Stage 2: Heater 1 shut down after 2 minutes of high temperature");
      }
    }
    
  } else {
    // Temperature is below threshold or sensor error
    
    // Reset high temperature timer
    if (highTempTimerActive) {
      highTempTimerActive = false;
      Serial.println("Temperature normalized - resetting timer");
    }
    
    // Re-enable heaters if temperature is acceptable and no sensor error
    if (Tt1 > 0) {  // Valid sensor reading
      if (!heater1Active) {
        digitalWrite(relay1, HIGH);
        heater1Active = true;
        Serial.println("Heater 1 re-enabled");
      }
      
      if (!heater2Active) {
        digitalWrite(relay2, HIGH);
        heater2Active = true;
        heater2ShutdownDueToTemp = false;
        Serial.println("Heater 2 re-enabled");
      }
    } else {
      // Sensor error - shut down heaters for safety
      digitalWrite(relay1, LOW);
      digitalWrite(relay2, LOW);
      heater1Active = false;
      heater2Active = false;
    }
  }
  
  // Update current states for backward compatibility
  currentState1 = heater1Active;
  currentState3 = heater2Active; // Note: currentState3 represents heater2 (relay2)
}

void handleAirFlowMonitoring() {
  float currentPressure = getPressure2(); // Only use BME2 sensor
  unsigned long currentTime = millis();
  
  bool currentMeasuredAirFlow = false;
  
  // Only calculate if sensor is working and we have a previous reading
  if (currentPressure > 0) {
    // Check if enough time has passed for pressure comparison (0.5 second)
    if (currentTime - lastPressureTime >= PRESSURE_SAMPLE_INTERVAL && previousPressure > 0) {
      float pressureDiff = currentPressure - previousPressure;
      
      // Air is flowing if pressure difference is more than 1 Pa
      // Air is not flowing if pressure difference is less than -1 Pa
      if (pressureDiff > 1.0) {
        currentMeasuredAirFlow = true;
      } else if (pressureDiff < -1.0) {
        currentMeasuredAirFlow = false;
      } else {
        // Pressure difference is between -1 and 1 Pa, keep previous state
        // This prevents rapid switching due to small fluctuations
        currentMeasuredAirFlow = airFlowStatus;
      }
      
      // Update previous pressure and time
      previousPressure = currentPressure;
      lastPressureTime = currentTime;
    } else if (previousPressure == 0) {
      // First reading - initialize previous pressure
      previousPressure = currentPressure;
      lastPressureTime = currentTime;
      // Keep current airflow status until we have a comparison
      currentMeasuredAirFlow = airFlowStatus;
    } else {
      // Not enough time has passed, keep current status
      currentMeasuredAirFlow = airFlowStatus;
    }
  }
  airFlowStatus = currentMeasuredAirFlow;
}

void printState(bool cS1, bool cS2, bool cS3){
  if (cS1 != previousState1){
    Serial.print("Status update: ");
    Serial.println(cS1 ? "Heater 1 (Relay1) ON" : "Heater 1 (Relay1) OFF");
    previousState1 = cS1;
  }
  if (cS2 != previousState2){
    Serial.print("Status update: ");
    Serial.println(cS2 ? "Heater 3 (Relay3) ON" : "Heater 3 (Relay3) OFF");
    previousState2 = cS2;
  }
  if (cS3 != previousState3){
    Serial.print("Status update: ");
    Serial.println(cS3 ? "Heater 2 (Relay2) ON" : "Heater 2 (Relay2) OFF");
    previousState3 = cS3;
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
      lcd.print("Heater 1:");
      lcd.setCursor(0, 1);
      lcd.print(heater1Active ? "ON" : "OFF");
      if (highTempTimerActive) {
        lcd.print(" (HT)");
      }
      break;
    case 5:
      lcd.setCursor(0, 0);
      lcd.print("Heater 2:");
      lcd.setCursor(0, 1);
      lcd.print(heater2Active ? "ON" : "OFF");
      if (heater2ShutdownDueToTemp) {
        lcd.print(" (HT)");
      }
      break;
    case 6:
      lcd.setCursor(0, 0);
      lcd.print("Heater 3:");
      lcd.setCursor(0, 1);
      lcd.print(heater3Active ? "ON" : "OFF");
      break;
    case 7:
      lcd.setCursor(0, 0);
      lcd.print("Air Flow:");
      lcd.setCursor(0, 1);
      lcd.print(airFlowStatus ? "Flowing" : "Not Flowing");
      break;
    case 8:
      lcd.setCursor(0, 0);
      lcd.print("Water Level");
      lcd.setCursor(0, 1);
      lcd.print(waterDetected ? "Detected" : "Not Detected");
      break;
  }
  lcdState = (lcdState + 1) % 9; // Updated to 9 states
}

void loop() {
  esp_task_wdt_reset(); // Reset watchdog
  
  // Check for relay state changes before updating sensors
  checkRelayStateChanges();
  
  // Update sensors efficiently (will be paused if relays just switched)
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

  // Print state changes
  printState(heater1Active, heater3Active, heater2Active);

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
          heater1Active,          // heater1_state (relay1)
          heater2Active,          // heater2_state (relay2)
          heater3Active           // heater3_state (relay3)
        );
    }
  }
  
  // Add small delay to prevent overwhelming the system
  delay(10);
}
