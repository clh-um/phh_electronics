// MQTT Management System
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <time.h>
#include "DeviceID.h"

// WiFi credentials
const char* ssid = "HLWONG 1575";
const char* password = "1Rw5@840";

// MQTT Broker settings
const char* mqtt_server = "5e44e2f2818640afae596ca821517f0e.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "esp32";
const char* mqtt_password = "1234567890aA";

// Baltimore CyberTrust Root Certificate
const char* root_ca = R"(
-----BEGIN CERTIFICATE-----
MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ
RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD
VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX
DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y
ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy
VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr
mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr
IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK
mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu
XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy
dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye
jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1
BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3
DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92
9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx
jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0
Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz
ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS
R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp
-----END CERTIFICATE-----
)";

// MQTT client
WiFiClientSecure espClient;
PubSubClient client(espClient);

// MQTT connection status tracking
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 5000; // 5 seconds between attempts
extern unsigned long lastPublishMillis;
extern const unsigned long publishInterval;

// External heater state variables
extern bool heater1Active;
extern bool heater2Active;
extern bool heater3Active;

// Device ID related constants
const int EEPROM_SIZE = 512;
const int DEVICE_ID_ADDRESS = 0;
const char* DEVICE_PREFIX = "HUM-";
const int DEVICE_ID_LENGTH = 10;

// MQTT topics
char deviceTopic[64];
char commandTopic[64];
extern phh::device::DeviceID deviceId;

void setupMQTT() {
    // Setup WiFi
    setupWiFi();
    setDateTime();

    // Generate or retrieve device ID
    if (!deviceId.initialize()) {
        #ifdef DEBUG
        Serial.println(F("Failed to initialize device ID!"));
        #endif
    }
    Serial.print("Device ID: ");
    Serial.println(deviceId.get());
    
    // Set up MQTT topics with device ID
    snprintf(deviceTopic, sizeof(deviceTopic), "device/%s/data", deviceId.get());
    snprintf(commandTopic, sizeof(commandTopic), "device/%s/command", deviceId.get());

    // Configure secure client
    espClient.setCACert(root_ca);
    espClient.setInsecure();

    // Initialize MQTT client
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback);
    client.setBufferSize(1024);
}

void setupWiFi() {
    Serial.println("\n=== WiFi Connection ===");
    Serial.println("Connecting to: ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    randomSeed(micros());

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        esp_task_wdt_reset();
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void setDateTime() {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    
    Serial.println("Waiting for NTP time sync: ");
        
    time_t now = time(nullptr);
    while (now < 8 * 3600 * 2) {
        esp_task_wdt_reset();
        delay(100);
        Serial.print(".");
        now = time(nullptr);
    }
    Serial.println("NTP time sync successful.");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    JsonDocument doc;
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    
    Serial.print("Message received: ");
    Serial.println(message);
    
    DeserializationError error = deserializeJson(doc, message);
    if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
    }

    if (doc["command"].is<const char*>() && doc["relay"].is<int>()) {
        String command = doc["command"].as<String>();
        int relayNum = doc["relay"].as<int>();
        bool state = doc["state"].as<bool>();

        Serial.print("Command received for relay ");
        Serial.print(relayNum);
        Serial.print(": ");
        Serial.println(state ? "ON" : "OFF");

        // Handle relay control based on relay number
        if (command == "relay_control") {
            switch (relayNum) {
                case 1:
                    digitalWrite(relay1, state ? HIGH : LOW);
                    heater1Active = state;
                    break;
                case 2:
                    digitalWrite(relay2, state ? HIGH : LOW);
                    heater2Active = state;
                    break;
                case 3:
                    digitalWrite(relay3, state ? HIGH : LOW);
                    heater3Active = state;
                    break;
                default:
                    Serial.println("Invalid relay number");
                    break;
            }

            // Publish status update with new structure
            publishSensorData(
                getThermocouple1(),     // heater_temp
                getThermocouple2(),     // final_temp
                getHumidity(),          // humidity
                getPressure1(),         // pressure1
                getPressure2(),         // pressure2
                waterDetected,          // water_detected
                heater1Active,          // heater1_state
                heater2Active,          // heater2_state
                heater3Active           // heater3_state
            );

            // Send confirmation message
            JsonDocument response;
            response["device_id"] = deviceId.get();
            response["type"] = "response";
            
            response["data"].to<JsonObject>();
            response["data"]["command"] = "relay_control";
            response["data"]["relay"] = relayNum;
            response["data"]["success"] = true;
            response["data"]["heater1_state"] = heater1Active;
            response["data"]["heater2_state"] = heater2Active;
            response["data"]["heater3_state"] = heater3Active;
            response["data"]["timestamp"] = time(nullptr);

            String responseStr;
            serializeJson(response, responseStr);
            char responseTopic[80];
            snprintf(responseTopic, sizeof(responseTopic), "%s/response", deviceTopic);
            client.publish(responseTopic, responseStr.c_str());
        }
    }
}

bool reconnectMQTT() {
    int attempts = 0;
    while (!client.connected() && attempts < 5) {
        attempts++;
        Serial.print("Attempting MQTT connection... (");
        Serial.print(attempts);
        Serial.print("/5) ");
        esp_task_wdt_reset();
        
        String clientId = String(deviceId.get()) + "-" + String(random(0xffff), HEX);
        
        if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
            Serial.println(" - Connected!");
            
            // Subscribe to device-specific command topic
            client.subscribe(commandTopic);
            Serial.print("Subscribed to: ");
            Serial.println(commandTopic);
            return true;
        } else {
            Serial.print(" - Failed, rc=");
            Serial.print(client.state());
            Serial.println(" - Retrying in 5 seconds");
            delay(5000);
        }
    }
    return false;
}

void publishSensorData(float heater_temp, float final_temp, float humidity, float pressure1, float pressure2, bool waterDetected, bool heater1State, bool heater2State, bool heater3State) {
    if (!client.connected()) {
        Serial.println("Cannot publish - MQTT not connected");
        return;
    }

    JsonDocument doc;
    
    doc["device_id"] = deviceId.get();
    doc["type"] = "data";

    doc["data"].to<JsonObject>();
    doc["data"]["humidity"] = humidity;
    doc["data"]["heater_temp"] = heater_temp;
    doc["data"]["final_temp"] = final_temp;
    doc["data"]["pressure1"] = pressure1;
    doc["data"]["pressure2"] = pressure2;
    doc["data"]["water_detected"] = waterDetected;
    doc["data"]["heater1_state"] = heater1State;
    doc["data"]["heater2_state"] = heater2State;
    doc["data"]["heater3_state"] = heater3State;

    // Warning flags
    doc["data"]["warnings"].to<JsonObject>();
    doc["data"]["warnings"]["heater_overtemp"] = heater_temp > Config::TEMP_THRESHOLD1;
    doc["data"]["warnings"]["final_overtemp"] = final_temp > Config::TEMP_THRESHOLD2;
    doc["data"]["warnings"]["water_low"] = !waterDetected;
    doc["data"]["warnings"]["no_airflow"] = !airFlowStatus;

    doc["data"]["timestamp"] = time(nullptr);

    String output;
    serializeJson(doc, output);
    
    if (client.publish(deviceTopic, output.c_str())) {
        Serial.println("Publish successful");
        Serial.println(output);
    } else {
        Serial.println("Publish failed");
    }
}

void handleMQTT() {
    // Check WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected. Reconnecting...");
        setupWiFi();
    }

    // Handle MQTT connection
    if (!client.connected()) {
        unsigned long now = millis();
        if (now - lastReconnectAttempt > reconnectInterval) {
            lastReconnectAttempt = now;
            if (reconnectMQTT()) {
                lastReconnectAttempt = 0;
            }
        }
    } else {
        client.loop();
    }
}
