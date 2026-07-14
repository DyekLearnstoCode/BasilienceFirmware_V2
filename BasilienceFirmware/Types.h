#ifndef TYPES_H
#define TYPES_H

struct SensorData
{
    float temperature = 0;
    float humidity = 0;
    float waterTemp = 0;
    float ec = 0;
    float tds = 0;
    float ecVoltage = 0;
    int ecRaw = 0;
    int ecRaw = 0;
    float ph = 0;
    float waterLevel = 0;
    bool ecReady = false;
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

#endif