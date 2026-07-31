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
    float temperature = NAN;
    float humidity    = NAN;
    float waterTemp   = NAN;

    // Electrical Conductivity
    float ec        = NAN;
    float tds       = NAN;
    float ecVoltage = NAN;
    int   ecRaw     = 0;

    // pH
    float ph          = NAN;
    int   phMilliVolts = 0;

    // Water Level
    float waterLevel = 0; // 0 is valid (empty tank) — kept as-is
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
    CANOPY_FAN,
    PELTIER,

    ACTUATOR_COUNT
};

inline const char* getActuatorName(Actuator a)
{
    switch(a)
    {
        case FOGGER: return "fogger";
        case GROW_LIGHT: return "growLight";
        case BLOWER: return "blower";
        case SOLENOID: return "solenoid";
        case GROW_PUMP: return "growPump";
        case BLOOM_PUMP: return "bloomPump";
        case PH_UP_PUMP: return "phUpPump";
        case PH_DOWN_PUMP: return "phDownPump";
        case CANOPY_FAN: return "canopyFan";
        case PELTIER: return "peltier";
        default: return "unknown";
    }
}

enum class ActuatorCommandState
{
    OFF,
    COMMAND_RECEIVED,
    VALIDATING,
    REJECTED,
    ACTIVATING,
    RUNNING,
    STOPPING
};

struct ActuatorCommand
{
    bool isPending = false;
    bool targetState = false;
    uint8_t speed = 100;
    String source = "";
    uint32_t timestamp = 0;
};

struct ActuatorStatus
{
    ActuatorCommandState state = ActuatorCommandState::OFF;
    bool running = false;
    uint8_t speed = 100;
    uint32_t startedAt = 0;
    String source = "";
    String reason = "";
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
    RequestSource source = RequestSource::NONE;
    OperationAction action = OperationAction::NONE;
    RequestState state = RequestState::IDLE;


    char reason[64] = "";

    unsigned long requestTimestamp = 0;
    unsigned long acceptedTimestamp = 0;
    unsigned long startedTimestamp = 0;
    unsigned long completedTimestamp = 0;
    unsigned long lastUpdatedTimestamp = 0;
    
};


enum class OperationSource
{
    NONE,
    MANUAL,
    AUTOMATIC
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