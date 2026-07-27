#ifndef DEBUG_MANAGER_H
#define DEBUG_MANAGER_H

#include <Types.h>
#include <Arduino.h>


class DebugManager
{
public:
    void begin();

    void update();

private:

    unsigned long lastPrintTime;

    uint8_t currentPage;
    void printSensors();
    void printActuators();
    void printHeader(const char* title);

    void printFloat(
        const char* label,
        float value,
        const char* unit,
        uint8_t decimals);

    void printInteger(
        const char* label,
        int value,
        const char* unit);

    void printBool(
        const char* label,
        bool value);

    void printSeparator();

    void printSystemStatus();

    void printAlerts();
    void printRTC();

    const char* getModeName(
    SystemMode mode);


};


#endif