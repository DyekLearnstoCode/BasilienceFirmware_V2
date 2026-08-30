#ifndef DEBUG_MANAGER_H
#define DEBUG_MANAGER_H

#include <Types.h>
#include <Arduino.h>


class DebugManager
{
public:
    void begin();

    void update();

    // Serial Monitor Focus Mode (see the automation resilience diagnostics
    // follow-up report). All three respond dynamically to the CURRENTLY
    // selected systemState.automationTestSubsystem every call - never a
    // compile-time flag - and always return true when it is NONE, so normal
    // full-system logging is completely unaffected. None of these decide
    // any control/automation behavior; they only decide what reaches
    // Serial.
    //
    // shouldPrintDebug: is this diagnostic CATEGORY relevant to the
    // currently isolated controller (or is no controller isolated)?
    bool shouldPrintDebug(DebugCategory category) const;

    // shouldPrintActuator: is THIS actuator's state-transition log relevant
    // to the currently isolated controller?
    bool shouldPrintActuator(Actuator actuator) const;

    // shouldPrintStateTransition: is a state change between these two
    // SystemModes relevant to the currently isolated controller? SAFETY_LOCK
    // and the common NORMAL/SENSOR_STABILIZATION resting states always pass
    // through, regardless of which controller (if any) is isolated - see the
    // implementation's own comment.
    bool shouldPrintStateTransition(SystemMode fromMode, SystemMode toMode) const;

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