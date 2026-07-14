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

struct SystemState
{
    bool manualMode = true;

    bool isMixing = false;
};

struct AlertState
{
    bool lowWater = false;

    bool highTemperature = false;

    bool ecLow = false;

    bool phOutOfRange = false;

    bool waterTempOutOfRange = false;
};

enum StartupState
{
    STARTUP_FOGGING,

    STARTUP_REST,

    NORMAL_OPERATION
};

enum MixingState
{
    MIX_NONE,

    MIX_DOSING,

    MIX_REFILL
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

#endif