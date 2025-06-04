#include "DeviceID.h"

namespace phh {
namespace device {

DeviceID::DeviceID() {
    memset(deviceId, 0, sizeof(deviceId));
}

bool DeviceID::initialize() {
    if (readFromEEPROM()) {
        #ifdef DEBUG
        Serial.println(F("\n=== Existing Device ID Found ==="));
        Serial.print(F("Device ID: "));
        Serial.println(deviceId);
        Serial.println(F("================================"));
        #endif
        return true;
    }

    return generateNew();
}

const char* DeviceID::get() const {
    return deviceId;
}

bool DeviceID::regenerate() {
    return generateNew();
}

bool DeviceID::readFromEEPROM() {
    EEPROM.begin(EEPROM_SIZE);

    // Read ID from EEPROM
    for (uint8_t i = 0; i < ID_LENGTH; i++) {
        deviceId[i] = EEPROM.read(ID_ADDRESS + i);
    }
    deviceId[ID_LENGTH] = '\0';

    EEPROM.end();

    // Validate ID format (HUM-XXXXX)
    return (strncmp(deviceId, "HUM-", 4) == 0);
}

bool DeviceID::generateNew() {
    static const char CHARSET[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    static const uint8_t CHARSET_SIZE = sizeof(CHARSET) - 1;

    #ifdef DEBUG
    Serial.println(F("Generating new Device ID..."));
    #endif

    // Set prefix
    strncpy(deviceId, "HUM-", 4);

    // Generate random suffix
    for (uint8_t i = 4; i < ID_LENGTH; i++) {
        deviceId[i] = CHARSET[random(CHARSET_SIZE)];
    }
    deviceId[ID_LENGTH] = '\0';

    // Save to EEPROM
    EEPROM.begin(EEPROM_SIZE);
    for (uint8_t i = 0; i <= ID_LENGTH; i++) {
        EEPROM.write(ID_ADDRESS + i, deviceId[i]);
    }
    EEPROM.commit();
    EEPROM.end();

    #ifdef DEBUG
    Serial.println(F("\n=== New Device ID Generated ==="));
    Serial.print(F("Device ID: "));
    Serial.println(deviceId);
    Serial.println(F("=============================="));
    #endif

    return true;
}

} // namespace device
} // namespace phh
