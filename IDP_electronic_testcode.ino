#include "max6675.h"
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

const int thermoDO = 19;
const int thermoCS1 = 23;
const int thermoCS2 = 26;
const int thermoCLK = 5;
const int lcdColumns = 16;
const int lcdRows = 2;
const int LiqSensorPin = 33; 
const int ledPin = 32;        // for testing only
const int relay1 = 25;
const int relay2 = 12;
const int relay3 = 14;
const int buzzer = 27;

#define DHTPIN 4
#define SEALEVELPRESSURE_HPA (1013.25)
#define DHTTYPE DHT22

// Configuration constants
struct Config {
  static constexpr float TEMP_THRESHOLD = 32.50;
  static constexpr float PRESSURE_THRESHOLD = 0.05;
  static constexpr unsigned long WATER_TIMEOUT = 10UL * 60UL * 1000UL;
  static constexpr unsigned long LCD_INTERVAL = 2000;
  static constexpr unsigned long SENSOR_INTERVAL = 2000;
  static constexpr unsigned long FAST_SENSOR_INTERVAL = 500;
  static constexpr unsigned long RETRY_INTERVAL = 100;
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

// System states for better organization
enum SystemState {
  NORMAL_OPERATION,
  WATER_WARNING,
  WATER_CRITICAL,
  AIRFLOW_WARNING,
  SENSOR_ERROR,
  EMERGENCY_SHUTDOWN
};

SystemState currentSystemState = NORMAL_OPERATION;

void IRAM_ATTR waterSensorInterrupt() {
  waterDetected = digitalRead(LiqSensorPin) == LOW;
  digitalWrite(ledPin, waterDetected ? HIGH : LOW);
  // Reset timer if water is detected
  if (waterDetected) {
    waterTimerRunning = false;      // Reset timer state
    waterNotDetectedStartTime = 0;  // Reset start time
    finalWarningSent = false;       // Reset final warning
  }
}

void setup() {
  Serial.begin(115200);

  // Enable watchdog timer
  esp_task_wdt_config_t config = {
    .timeout_ms = 30000,  // 30 seconds
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };
  esp_task_wdt_init(&config); // 30 second timeout
  esp_task_wdt_add(NULL);

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
  waterDetected = digitalRead(LiqSensorPin) == LOW;
  if (!waterDetected) {
    waterNotDetectedStartTime = millis();  // Start timer if no water detected
    waterTimerRunning = true;
  }

  Serial.println("-- Smart Prefilled Humidifier Heater System Ready --");
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

void updateWaterSensorReading() {
  unsigned long currentTime = millis();
  
  // Check water sensor more frequently than other sensors
  if (currentTime - lastWaterSensorCheck >= WATER_SENSOR_CHECK_INTERVAL) {
    
    // Read the sensor directly (like in original code)
    bool currentReading = digitalRead(LiqSensorPin) == LOW;
    
    // Debounce the reading by checking twice with a small delay
    delay(10);
    bool confirmedReading = digitalRead(LiqSensorPin) == LOW;
    
    // Only update if both readings match (debouncing)
    if (currentReading == confirmedReading) {
      waterDetected = confirmedReading;
      digitalWrite(ledPin, waterDetected ? HIGH : LOW);
      
      // Reset timer if water is detected (like in original)
      if (waterDetected) {
        waterTimerRunning = false;
        waterNotDetectedStartTime = 0;
        finalWarningSent = false;
      }
    }
    
    lastWaterSensorCheck = currentTime;
  }
}

void handleWaterSafety() {
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
  // Non-blocking final warning implementation
  static unsigned long warningStartTime = 0;
  static int warningCount = 0;
  
  if (warningStartTime == 0) {
    warningStartTime = millis();
    warningCount = 0;
  }
  
  // This could be made fully non-blocking with a state machine
  for (int i = 0; i < 5; i++) {
    tone(buzzer, NOTE_B7);
    delay(500);
    noTone(buzzer);
    delay(500);
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

  // Get sensor readings
  float Tt1 = getThermocouple1();
  float Tt2 = getThermocouple2();
  
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
  
  // Only calculate if both sensors are working
  if (BME1p > 0 && BME2p > 0) {
    float pressureDiff = abs(BME2p - BME1p);
    airFlowStatus = (pressureDiff > Config::PRESSURE_THRESHOLD);
  } else {
    airFlowStatus = false; // Assume no airflow if sensors fail
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
      lcd.print("Humidity:");
      lcd.setCursor(0, 1);
      lcd.print(h >= 0 ? h : 0);
      lcd.print("%");
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("Heater Temp:");
      lcd.setCursor(0, 1);
      lcd.print(Tt1 != -999.0 ? Tt1 : 0);
      lcd.print("*C");
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("Final Temp:");
      lcd.setCursor(0, 1);
      lcd.print(Tt2 != -999.0 ? Tt2 : 0);
      lcd.print("*C");
      break;
    case 3:
      lcd.setCursor(0, 0);
      lcd.print("Heater 1 & 2:");
      lcd.setCursor(0, 1);
      lcd.print(h1Status);
      break;
    case 4:
      lcd.setCursor(0, 0);
      lcd.print("Heater 3:");
      lcd.setCursor(0, 1);
      lcd.print(h3Status);
      break;
    case 5:
      lcd.setCursor(0, 0);
      lcd.print("Air Flow:");
      lcd.setCursor(0, 1);
      lcd.print(airFlowStatus ? "Flowing" : "Not Flowing");
      break;
    case 6:
      lcd.setCursor(0, 0);
      lcd.print("Water Level");
      lcd.setCursor(0, 1);
      lcd.print(waterDetected ? "Detected" : "Not Detected");
      break;
  }
  lcdState = (lcdState + 1) % 7;
}

void loop() {
  esp_task_wdt_reset(); // Reset watchdog

  updateWaterSensorReading();
  
  // Update sensors efficiently
  updateSensorsIfNeeded();

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

  // Publish sensor data periodically
  unsigned long currentMillis = millis();
  if (currentMillis - lastPublishMillis >= publishInterval) {
    lastPublishMillis = currentMillis;
    publishSensorData(
      getHumidity(),
      getThermocouple1(),
      getThermocouple2(),
      getPressure1(),
      getPressure2(),
      waterDetected,
      currentState1,
      currentState1, // Both heater 1&2 use the same state
      currentState2  // Heater 3
    );
  }
  // Optional: Add small delay to prevent overwhelming the system
  delay(10);
}
