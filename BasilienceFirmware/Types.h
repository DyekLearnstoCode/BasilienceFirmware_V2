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

    // Diagnostics only: raw HC-SR04 measured distance in centimeters, before
    // percentage conversion. Never consumed by automation/alerts/safety.
    float waterLevelDistanceCm = NAN;
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

    INVALID_EC,

    SUBSYSTEM_LOCKED,

    RESERVOIR_FULL
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
    CIRCULATION_PUMP,

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
        case CIRCULATION_PUMP: return "circulationPump";
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
    String strategy = "";
    String reason = "";
    uint32_t timestamp = 0;
    // One-shot manual-override intent for THIS command only (see
    // ActuatorManager::validateCommand). Never persists past the command it
    // arrived on - a later command with this unset/false returns to normal
    // soft-rule enforcement.
    bool overrideRequested = false;
};

struct ActuatorStatus
{
    ActuatorCommandState state = ActuatorCommandState::OFF;
    bool running = false;
    uint8_t speed = 100;
    uint32_t startedAt = 0;
    String source = "";
    String strategy = "";
    String reason = "";
    // Mirrors the overrideRequested carried by the command currently
    // occupying this actuator, so validateCommand's continuous RUNNING-state
    // re-check (which reads status, not the transient command) can honor the
    // same override for the actuator's whole run, and so actuatorStatus can
    // publish it for the app to display.
    bool overrideActive = false;
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

enum ECDirection
{
    EC_NONE,
    EC_RAISE,
    EC_DILUTE
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

    bool phSubsystemLocked = false;
    bool ecSubsystemLocked = false;
    bool refillSubsystemLocked = false;
    bool coolingSubsystemLocked = false;

    bool reservoirLocked = false;

    bool forceRefill = false;

    bool resetSafetyLock = false;

    // Developer Mode physical sensor diagnostics
    bool sensorTestEnabled = false;
    unsigned long sensorTestStartTime = 0;

    // Remote Mocking
    bool mockSensorsEnabled = false;
    SensorData mockSensors;
    bool mockApplyPending = false;

    // True once the effective sensor source (mock vs. physical) has been
    // confirmed at least once from Firebase since boot. Shared between
    // FirebaseManager (which resolves it) and SensorManager/AlertManager
    // (which must not act on physical readings until it is true) so a
    // reboot/brownout can't let temporary physical readings drive automation
    // before a previously-enabled mock mode is restored.
    bool sensorSourceResolved = false;

    // Armed by AutomationManager::completeCurrentOperation() the instant an
    // automatic PH_UP/PH_DOWN/EC_CORRECTION reaches COMPLETED locally.
    // Released by FirebaseManager as soon as that COMPLETED state is
    // confirmed published to RTDB, or by AutomationManager after a bounded
    // local grace period if Firebase cannot be reached - so Fogger/Blower
    // resume waits for the cloud when possible but is never blocked by it.
    bool chemistryFoggingHoldActive = false;
    unsigned long chemistryFoggingHoldStartTime = 0;

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

    float minPH = MIN_PH;

    float maxPH = MAX_PH;

    float phTargetMin = PH_TARGET_MIN;

    float phTargetMax = PH_TARGET_MAX;

    //==================================================
    // EC
    //==================================================

    unsigned long ecDoseTime = 0;

    uint8_t ecAttempts = 0;

    float minEC = MIN_EC;

    float maxEC = MAX_EC;

    float ecTargetMin = EC_TARGET_MIN;

    float ecTargetMax = EC_TARGET_MAX;

    ECDirection ecDirection = EC_NONE;

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
    // Target (acceptable) ranges
    //
    // What "in range" means for Monitoring, alerts and Reports. Distinct from
    // the actuator control/hysteresis thresholds further below - see Config.h.
    // pH and EC already have their canonical pairs above (minPH/maxPH,
    // minEC/maxEC) and are not duplicated here.
    //==================================================

    float minAirTemp = TARGET_MIN_AIR_TEMP;
    float maxAirTemp = TARGET_MAX_AIR_TEMP;

    float minHumidity = TARGET_MIN_HUMIDITY;
    float maxHumidity = TARGET_MAX_HUMIDITY;

    float minWaterTemp = TARGET_MIN_WATER_TEMP;
    float maxWaterTemp = TARGET_MAX_WATER_TEMP;

    float minWaterLevel = TARGET_MIN_WATER_LEVEL;
    float maxWaterLevel = TARGET_MAX_WATER_LEVEL;

    //==================================================
    // Temperature
    //==================================================

    float highAirTemp =
        HIGH_AIR_TEMP;

    float airTempRelease =
        AIR_TEMP_RELEASE;

    float highHumidity =
        HIGH_HUMIDITY;

    float humidityRelease =
        HUMIDITY_RELEASE;

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

    bool lowAirTemperature = false;

    bool ecLow = false;

    bool ecHigh = false;

    bool phOutOfRange = false;

    bool phLow = false;

    bool phHigh = false;

    // Above maxWaterTemp. Name kept for compatibility with the existing Cloud
    // Function producer and the Android readers that already consume it.
    bool waterTempOutOfRange = false;
    // Below minWaterTemp - new directional counterpart.
    bool waterTempLow = false;

    // Humidity had no alert flags at all before target ranges existed.
    bool humidityLow = false;
    bool humidityHigh = false;

    // Target-range classification for water level. Deliberately separate from
    // lowWater above, which is the CONTROL signal that triggers automatic
    // refill and must keep following refillStartLevel.
    bool waterLevelLow = false;
    bool waterLevelHigh = false;

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
