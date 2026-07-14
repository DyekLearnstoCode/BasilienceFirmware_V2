#ifndef DEBUG_MANAGER_H
#define DEBUG_MANAGER_H

class DebugManager
{
public:
    void begin();

    void update();

private:
    unsigned long lastPrintTime;

    bool showSensorPage;

    // ============================
    // Pages
    // ============================

    void printSensors();

    void printActuators();

    // ============================
    // Helpers
    // ============================

    void printHeader(const char *title);

    void printFloat(
        const char *label,
        float value,
        const char *unit,
        uint8_t decimals);

    void printInteger(
        const char *label,
        int value,
        const char *unit);

    void printBool(
        const char *label,
        bool value);

    void printSeparator();
};

#endif