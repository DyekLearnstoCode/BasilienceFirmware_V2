#pragma once

#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>
#include "Config.h"

//==================================================
// Sensor Data
//==================================================

struct SensorData
{
    float temperature = 0;
    float humidity = 0;
    float waterTemp = 0;

    // Electrical Conductivity
    float ec = 0;
    float tds = 0;
    float ecVoltage = 0;
    int ecRaw = 0;

    // pH
    float ph = 0;
    int phMilliVolts = 0;

    // Water Level
    float waterLevel = 0;
};



//==================================================
// System Modes
//==================================================

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

enum class SafetyResult
{
    SAFE,

    LOW_WATER,

    RESERVOIR_LOCK,

    SAFETY_LOCKED,

    SENSOR_FAULT,

    HIGH_WATER_TEMP,

    INVALID_PH,

    INVALID_EC
};


//==================================================
// Actuators
//==================================================

enum Actuator
{
    // SSR
    FOGGER,
    GROW_LIGHT,

    // MOSFET
    BLOWER,
    SOLENOID,

    // Dosing Pumps
    GROW_PUMP,
    BLOOM_PUMP,
    PH_UP_PUMP,
    PH_DOWN_PUMP,

    // Temperature
    WATER_HEATER,
    PELTIER,

    ACTUATOR_COUNT
};

//==================================================
// pH
//==================================================

enum PHDirection
{
    PH_NONE,
    PH_UP,
    PH_DOWN
};


//==================================================
// Operation Protocol
//==================================================

enum class OperationType
{
    NONE,

    REFILL,

    PH_UP,
    PH_DOWN,

    EC_CORRECTION,

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
    IDLE,
    PENDING,
    ACCEPTED,
    RUNNING,
    COMPLETED,
    REJECTED,
    FAILED
};

//==================================================
// Operation Request
//==================================================

struct OperationRequest
{
    uint16_t requestId = 0;

    OperationType operation = OperationType::NONE;
    OperationAction action = OperationAction::NONE;
    RequestSource source = RequestSource::NONE;
    RequestState state = RequestState::IDLE;

    char reason[64] = "";

    unsigned long requestTimestamp = 0;
    unsigned long acceptedTimestamp = 0;
    unsigned long startedTimestamp = 0;
    unsigned long completedTimestamp = 0;
    unsigned long lastUpdatedTimestamp = 0;
};

//==================================================
// Runtime System State
//==================================================

enum class CorrectionMode
{
    NONE,
    AUTOMATIC,
    MANUAL
};
    
struct SystemState
{
    //==================================================
    // Runtime
    //==================================================

    bool manualMode = false;

    bool settingsLoaded = false;

    bool wifiConnected = false;

    bool firebaseConnected = false;

    bool syncRTC = false;

    bool safetyLock = false;

    bool reservoirLocked = false;

    bool forceRefill = false;

    bool resetSafetyLock = false;

    //==================================================
    // Operations
    //==================================================

    OperationRequest operationRequest;

    uint16_t lastProcessedRequestId = 0;

    //==================================================
    // State Machine
    //==================================================

    SystemMode currentMode =
        SENSOR_STABILIZATION;

    unsigned long stateStartTime = 0;

    unsigned long dosingStartTime = 0;

    CorrectionMode correctionMode =
        CorrectionMode::NONE;

    bool firstCorrectionCycle = true;

    //==================================================
    // pH
    //==================================================

    PHDirection phDirection =
        PH_NONE;

    unsigned long phDoseTime = 0;

    uint8_t phAttempts = 0;

    float minPH = 5.5f;

    float maxPH = 6.5f;

    //==================================================
    // EC
    //==================================================

    unsigned long ecDoseTime = 0;

    uint8_t ecAttempts = 0;

    float minEC = 1.0f;

    //==================================================
    // Grow Light Schedule
    //==================================================

    uint8_t lightOnHour = 6;
    uint8_t lightOnMinute = 0;

    uint8_t lightOffHour = 22;
    uint8_t lightOffMinute = 0;

    //==================================================
    // Reservoir
    //==================================================

    float refillStartLevel =
        REFILL_START_LEVEL;

    float refillStopLevel =
        REFILL_STOP_LEVEL;

    //==================================================
    // Temperature
    //==================================================

    float highAirTemp =
        HIGH_AIR_TEMP;

    float highWaterTemp =
        HIGH_WATER_TEMP;

    float coolerOffTemp =
        COOLER_OFF_TEMP;

    float hotFogTemperature = 30.0f;

    float coldFogTemperature = 20.0f;
};

//==================================================
// Alerts
//==================================================

struct AlertState
{
    bool lowWater = false;

    bool highTemperature = false;

    bool ecLow = false;

    bool phOutOfRange = false;

    bool waterTempOutOfRange = false;

    bool sensorFault = false;
};

//==================================================
// Actuator Runtime
//==================================================

struct ActuatorState
{
    bool states[ACTUATOR_COUNT] = { false };
};

//==================================================
// Telemetry
//==================================================

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