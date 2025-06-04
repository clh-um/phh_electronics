#ifndef PHH_DEVICE_ID_H
#define PHH_DEVICE_ID_H

#include <Arduino.h>
#include <EEPROM.h>

namespace phh {
namespace device {

// Constants
constexpr uint8_t ID_LENGTH = 9;
constexpr uint16_t ID_ADDRESS = 0;
constexpr uint16_t EEPROM_SIZE = 512;

class DeviceID {
public:
    DeviceID();

    // Initialize device ID - returns true if successful
    bool initialize();

    // Get current device ID
    const char* get() const;

    // Force generate new ID
    bool regenerate();

private:
    bool readFromEEPROM();
    bool generateNew();

    char deviceId[ID_LENGTH + 1];  // +1 for null terminator
};

} // namespace device
} // namespace phh

#endif // PHH_DEVICE_ID_H
