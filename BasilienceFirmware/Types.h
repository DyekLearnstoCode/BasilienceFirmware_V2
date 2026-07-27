#pragma once

#include "Config.h"

#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

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

enum PHDirection
{
    PH_NONE,
    PH_UP,
    PH_DOWN
};

enum class OperationType
{
    NONE,

    PH_UP,
    PH_DOWN,

    GROW_PUMP,
    BLOOM_PUMP,

    REFILL,

    FOGGER,
    CANOPY_FAN,

    PELTIER,

    GROW_LIGHT,

    RESTART_ESP,

    RESET_SAFETY
};

enum class OperationAction
{
    NONE,

    START,
    STOP,

    ENABLE,
    DISABLE,

    EXECUTE
};

enum class RequestSource
{
    NONE,
    MANUAL,
    AUTOMATIC
};

enum class RequestState
{
    IDLE,       // No active request.
    ACCEPTED,
    PENDING,
    RUNNING,
    COMPLETED,
    REJECTED,
    FAILED
};


struct OperationRequest
{
    uint16_t requestId;

    OperationType operation;
    OperationAction action;
    RequestSource source;
    RequestState state;

    char reason[64];

    unsigned long requestTimestamp;
    unsigned long acceptedTimestamp;
    unsigned long startedTimestamp;
    unsigned long completedTimestamp;
    unsigned long lastUpdatedTimestamp;
};

struct SystemState
{
    bool manualMode = false;
    
    OperationRequest operationRequest;
    uint16_t lastProcessedRequestId = 0;

    bool reservoirLocked = false;

    SystemMode currentMode =
        SENSOR_STABILIZATION;

    unsigned long stateStartTime = 0;

    unsigned long dosingStartTime = 0;

    PHDirection phDirection =
        PH_NONE;

    unsigned long phDoseTime = 0;

    unsigned long ecDoseTime = 0;

    uint8_t phAttempts = 0;

    uint8_t ecAttempts = 0;
    
    uint8_t lightOnHour = 6;
    uint8_t lightOnMinute = 0;

    uint8_t lightOffHour = 22;
    uint8_t lightOffMinute = 0;

    bool settingsLoaded = false;

    bool wifiConnected = false;

    bool firebaseConnected = false;

    float minPH = 5.5f;
    float maxPH = 6.5f;

    float minEC = 1.0f;

    bool forceRefill = false;
    bool resetSafetyLock = false;

    bool syncRTC = false;
    bool safetyLock = false;

    float refillStartLevel =
        REFILL_START_LEVEL;

    float refillStopLevel =
        REFILL_STOP_LEVEL;

    float highAirTemp =
    HIGH_AIR_TEMP;

    float highWaterTemp =
    HIGH_WATER_TEMP;

    float coolerOffTemp =
        COOLER_OFF_TEMP;
};

struct AlertState
{
    bool lowWater = false;
    bool highTemperature = false;
    bool ecLow = false;
    bool phOutOfRange = false;
    bool waterTempOutOfRange = false;
    bool sensorFault = false;
};

struct ActuatorState
{
    bool states[ACTUATOR_COUNT] = { false };
};

struct ActuatorTelemetry
{
    bool fogger = false;

    bool growLight = false;

    bool blower = false;

    bool solenoid = false;

    bool growPump = false;

    bool bloomPump = false;

    bool phUpPump = false;

    bool phDownPump = false;

    bool peltier = false;
};



#endif