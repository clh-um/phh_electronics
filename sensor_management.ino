// Sensor readings structure
struct SensorReadings {
  float humidity;
  float temperature;
  float thermocouple1;
  float thermocouple2;
  float pressure1;
  float pressure2;
  unsigned long lastUpdate;
  bool validReadings[6]; // Track which readings are valid
};

// Global sensor readings instance
SensorReadings sensorData = {0, 0, 0, 0, 0, 0, 0, {false, false, false, false, false, false}};

// Configuration constants
const unsigned long SENSOR_INTERVAL = 2000;        // Read sensors every 2 seconds
const unsigned long FAST_SENSOR_INTERVAL = 500;    // Fast reading for critical sensors
const unsigned long RETRY_INTERVAL = 100;          // Time between retries

// Sensor indices for the validReadings array
enum SensorIndex {
  HUMIDITY_IDX = 0,
  TEMPERATURE_IDX = 1,
  THERMOCOUPLE1_IDX = 2,
  THERMOCOUPLE2_IDX = 3,
  PRESSURE1_IDX = 4,
  PRESSURE2_IDX = 5
};

void updateSensorsIfNeeded() {
  unsigned long currentTime = millis();
  
  // Determine update interval based on system state
  unsigned long updateInterval = SENSOR_INTERVAL;
  
  // Use faster updates if water not detected or in critical state
  if (!waterDetected || !airFlowStatus) {
    updateInterval = FAST_SENSOR_INTERVAL;
  }
  
  // Check if it's time to update sensors
  if (currentTime - sensorData.lastUpdate >= updateInterval) {
    updateAllSensors();
    sensorData.lastUpdate = currentTime;
  }
}

void updateAllSensors() {
  // Update humidity (DHT22)
  updateHumiditySensor();
  
  // Update temperature (DHT22)
  updateTemperatureSensor();
  
  // Update thermocouples (critical for heater control)
  updateThermocoupleSensors();
  
  // Update pressure sensors (for airflow detection)
  updatePressureSensors();
  
  // Log sensor status
  logSensorStatus();
}

void updateHumiditySensor() {
  static unsigned long lastAttempt = 0;
  static int retryCount = 0;
  const int MAX_RETRIES = 3;
  
  unsigned long currentTime = millis();
  
  // Don't retry too frequently
  if (currentTime - lastAttempt < RETRY_INTERVAL && retryCount > 0) {
    return;
  }
  
  float newHumidity = dht.readHumidity();
  
  if (!isnan(newHumidity) && newHumidity >= 0 && newHumidity <= 100) {
    sensorData.humidity = newHumidity;
    sensorData.validReadings[HUMIDITY_IDX] = true;
    retryCount = 0; // Reset retry counter on success
  } else {
    sensorData.validReadings[HUMIDITY_IDX] = false;
    retryCount++;
    
    if (retryCount >= MAX_RETRIES) {
      Serial.println("Warning: DHT humidity reading failed after retries");
      retryCount = 0; // Reset to try again next cycle
    }
  }
  
  lastAttempt = currentTime;
}

void updateTemperatureSensor() {
  static unsigned long lastAttempt = 0;
  static int retryCount = 0;
  const int MAX_RETRIES = 3;
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastAttempt < RETRY_INTERVAL && retryCount > 0) {
    return;
  }
  
  float newTemp = dht.readTemperature();
  
  if (!isnan(newTemp) && newTemp >= -40 && newTemp <= 80) { // Reasonable range
    sensorData.temperature = newTemp;
    sensorData.validReadings[TEMPERATURE_IDX] = true;
    retryCount = 0;
  } else {
    sensorData.validReadings[TEMPERATURE_IDX] = false;
    retryCount++;
    
    if (retryCount >= MAX_RETRIES) {
      Serial.println("Warning: DHT temperature reading failed after retries");
      retryCount = 0;
    }
  }
  
  lastAttempt = currentTime;
}

void updateThermocoupleSensors() {
  // Thermocouple 1
  static unsigned long lastAttempt1 = 0;
  static int retryCount1 = 0;
  
  // Thermocouple 2
  static unsigned long lastAttempt2 = 0;
  static int retryCount2 = 0;
  
  const int MAX_RETRIES = 3;
  const float MIN_TEMP = -10.0;
  const float MAX_TEMP = 100.0;
  
  unsigned long currentTime = millis();
  
  // Update Thermocouple 1
  if (currentTime - lastAttempt1 >= RETRY_INTERVAL || retryCount1 == 0) {
    float temp1 = thermocouple1.readCelsius();
    
    if (!isnan(temp1) && temp1 >= MIN_TEMP && temp1 <= MAX_TEMP) {
      sensorData.thermocouple1 = temp1;
      sensorData.validReadings[THERMOCOUPLE1_IDX] = true;
      retryCount1 = 0;
    } else {
      sensorData.validReadings[THERMOCOUPLE1_IDX] = false;
      retryCount1++;
      
      if (retryCount1 >= MAX_RETRIES) {
        Serial.println("Warning: Thermocouple 1 reading failed after retries");
        retryCount1 = 0;
      }
    }
    lastAttempt1 = currentTime;
  }
  
  // Update Thermocouple 2
  if (currentTime - lastAttempt2 >= RETRY_INTERVAL || retryCount2 == 0) {
    float temp2 = thermocouple2.readCelsius();
    
    if (!isnan(temp2) && temp2 >= MIN_TEMP && temp2 <= MAX_TEMP) {
      sensorData.thermocouple2 = temp2;
      sensorData.validReadings[THERMOCOUPLE2_IDX] = true;
      retryCount2 = 0;
    } else {
      sensorData.validReadings[THERMOCOUPLE2_IDX] = false;
      retryCount2++;
      
      if (retryCount2 >= MAX_RETRIES) {
        Serial.println("Warning: Thermocouple 2 reading failed after retries");
        retryCount2 = 0;
      }
    }
    lastAttempt2 = currentTime;
  }
}

void updatePressureSensors() {
  // BME280 Sensor 1
  static unsigned long lastAttempt1 = 0;
  static int retryCount1 = 0;
  
  // BME280 Sensor 2
  static unsigned long lastAttempt2 = 0;
  static int retryCount2 = 0;
  
  const int MAX_RETRIES = 3;
  const float MIN_PRESSURE = 800.0;  // hPa
  const float MAX_PRESSURE = 1200.0; // hPa
  
  unsigned long currentTime = millis();
  
  // Update BME280 1
  if (currentTime - lastAttempt1 >= RETRY_INTERVAL || retryCount1 == 0) {
    float pressure1 = bme1.readPressure() / 100.0F; // Convert to hPa
    
    if (!isnan(pressure1) && pressure1 >= MIN_PRESSURE && pressure1 <= MAX_PRESSURE) {
      sensorData.pressure1 = pressure1;
      sensorData.validReadings[PRESSURE1_IDX] = true;
      retryCount1 = 0;
    } else {
      sensorData.validReadings[PRESSURE1_IDX] = false;
      retryCount1++;
      
      if (retryCount1 >= MAX_RETRIES) {
        Serial.println("Warning: BME280-1 pressure reading failed after retries");
        retryCount1 = 0;
      }
    }
    lastAttempt1 = currentTime;
  }
  
  // Update BME280 2
  if (currentTime - lastAttempt2 >= RETRY_INTERVAL || retryCount2 == 0) {
    float pressure2 = bme2.readPressure() / 100.0F; // Convert to hPa
    
    if (!isnan(pressure2) && pressure2 >= MIN_PRESSURE && pressure2 <= MAX_PRESSURE) {
      sensorData.pressure2 = pressure2;
      sensorData.validReadings[PRESSURE2_IDX] = true;
      retryCount2 = 0;
    } else {
      sensorData.validReadings[PRESSURE2_IDX] = false;
      retryCount2++;
      
      if (retryCount2 >= MAX_RETRIES) {
        Serial.println("Warning: BME280-2 pressure reading failed after retries");
        retryCount2 = 0;
      }
    }
    lastAttempt2 = currentTime;
  }
}

// Helper functions to get sensor data safely
float getHumidity() {
  return sensorData.validReadings[HUMIDITY_IDX] ? sensorData.humidity : -1.0;
}

float getTemperature() {
  return sensorData.validReadings[TEMPERATURE_IDX] ? sensorData.temperature : -999.0;
}

float getThermocouple1() {
  return sensorData.validReadings[THERMOCOUPLE1_IDX] ? sensorData.thermocouple1 : -999.0;
}

float getThermocouple2() {
  return sensorData.validReadings[THERMOCOUPLE2_IDX] ? sensorData.thermocouple2 : -999.0;
}

float getPressure1() {
  return sensorData.validReadings[PRESSURE1_IDX] ? sensorData.pressure1 : -1.0;
}

float getPressure2() {
  return sensorData.validReadings[PRESSURE2_IDX] ? sensorData.pressure2 : -1.0;
}

bool areCriticalSensorsWorking() {
  // Thermocouples are critical for heater safety
  return sensorData.validReadings[THERMOCOUPLE1_IDX] && 
         sensorData.validReadings[THERMOCOUPLE2_IDX];
}

bool areAllSensorsWorking() {
  for (int i = 0; i < 6; i++) {
    if (!sensorData.validReadings[i]) {
      return false;
    }
  }
  return true;
}

void logSensorStatus() {
  static unsigned long lastLog = 0;
  const unsigned long LOG_INTERVAL = 30000; // Log every 30 seconds
  
  if (millis() - lastLog >= LOG_INTERVAL) {
    Serial.println("=== Sensor Status ===");
    Serial.print("Humidity: "); 
    Serial.print(sensorData.validReadings[HUMIDITY_IDX] ? "OK" : "FAIL");
    Serial.print(" ("); Serial.print(sensorData.humidity); Serial.println("%)");
    
    Serial.print("Temperature: "); 
    Serial.print(sensorData.validReadings[TEMPERATURE_IDX] ? "OK" : "FAIL");
    Serial.print(" ("); Serial.print(sensorData.temperature); Serial.println("°C)");
    
    Serial.print("Thermocouple1: "); 
    Serial.print(sensorData.validReadings[THERMOCOUPLE1_IDX] ? "OK" : "FAIL");
    Serial.print(" ("); Serial.print(sensorData.thermocouple1); Serial.println("°C)");
    
    Serial.print("Thermocouple2: "); 
    Serial.print(sensorData.validReadings[THERMOCOUPLE2_IDX] ? "OK" : "FAIL");
    Serial.print(" ("); Serial.print(sensorData.thermocouple2); Serial.println("°C)");
    
    Serial.print("Pressure1: "); 
    Serial.print(sensorData.validReadings[PRESSURE1_IDX] ? "OK" : "FAIL");
    Serial.print(" ("); Serial.print(sensorData.pressure1); Serial.println(" hPa)");
    
    Serial.print("Pressure2: "); 
    Serial.print(sensorData.validReadings[PRESSURE2_IDX] ? "OK" : "FAIL");
    Serial.print(" ("); Serial.print(sensorData.pressure2); Serial.println(" hPa)");
    
    Serial.println("====================");
    lastLog = millis();
  }
}