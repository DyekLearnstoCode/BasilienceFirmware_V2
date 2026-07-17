#ifndef TYPES_H
#define TYPES_H

struct SensorData
{
    float temperature = 0;
    float humidity = 0;
    float waterTemp = 0;

    // ==========================
    // EC
    // ==========================

    float ec = 0;
    float tds = 0;
    float ecVoltage = 0;
    int ecRaw = 0;

    // ==========================
    // pH
    // ==========================

    float ph = 0;
    int phMilliVolts = 0;

    // ==========================
    // Water Level
    // ==========================

    float waterLevel = 0;
};

enum SystemMode
{
    SENSOR_STABILIZATION,
    STARTUP,
    NORMAL,
    REFILLING,
    DOSING_PH,
    STABILIZING_PH,
    DOSING_EC,
    STABILIZING_EC,
    SAFETY_LOCK
};

enum Actuator
{
    // ======================================================
    // SSR Outputs
    // ======================================================

    FOGGER,
    GROW_LIGHT,

    // ======================================================
    // MOSFET Outputs
    // ======================================================

    BLOWER,
    SOLENOID,

    // ======================================================
    // Peristaltic Pumps
    // ======================================================

    GROW_PUMP,
    BLOOM_PUMP,
    PH_UP_PUMP,
    PH_DOWN_PUMP,

    // ======================================================
    // Temperature
    // ======================================================

    WATER_HEATER,
    PELTIER,

    ACTUATOR_COUNT
};

struct SystemState
{
    bool manualMode = false;

    bool reservoirLocked = false;

    SystemMode currentMode =
        SENSOR_STABILIZATION;

    unsigned long stateStartTime = 0;

    bool startupCompleted = false;
};

struct AlertState
{
    bool lowWater = false;

    bool highTemperature = false;

    bool ecLow = false;

    bool phOutOfRange = false;

    bool waterTempOutOfRange = false;
};

struct ActuatorState
{
    bool states[ACTUATOR_COUNT] = { false };
};

#endif