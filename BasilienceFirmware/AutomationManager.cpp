#include "AutomationManager.h"

#include "Config.h"
#include "Globals.h"

void AutomationManager::begin()
{
    startupPhase = STARTUP_FOG_ON;

    fogCycleOn = true;

    systemState.currentMode =
        SENSOR_STABILIZATION;

    systemState.stateStartTime =
        millis();

    fogTimerStart = millis();
}

void AutomationManager::update()
{
    //--------------------------------------------------
    // Operation lifecycle
    //--------------------------------------------------

    if(systemState.operationRequest.state ==
       RequestState::ACCEPTED)
    {
        systemState.operationRequest.state =
            RequestState::RUNNING;

        systemState.operationRequest.startedTimestamp =
            millis();

        systemState.operationRequest.lastUpdatedTimestamp =
            systemState.operationRequest.startedTimestamp;

        return;
    }

    if(systemState.operationRequest.state ==
       RequestState::RUNNING)
    {
        processOperationRequest();
        return;
    }

    //--------------------------------------------------
    // RTC synchronization
    //--------------------------------------------------

    if(systemState.syncRTC)
    {
        systemState.syncRTC = false;

        syncRTCFromFirebase();

        return;
    }

    //--------------------------------------------------
    // Manual mode
    //--------------------------------------------------

    if(systemState.manualMode)
    {
        // Abort any running automated operations when manual override mode is enabled
        if(systemState.operationRequest.state == RequestState::RUNNING)
        {
            failCurrentOperation("Aborted: manual override mode activated");
        }
        
        // Force the FSM state back to NORMAL if it is in an active dosing/refilling operational state
        if(systemState.currentMode != NORMAL && systemState.currentMode != SENSOR_STABILIZATION && systemState.currentMode != SAFETY_LOCK)
        {
            changeState(NORMAL);
        }
    }

    //--------------------------------------------------
    // State Machine
    //--------------------------------------------------

    processCurrentState();

} //Core Framework

void AutomationManager::processCurrentState()
{
    switch(systemState.currentMode)
    {
        case SENSOR_STABILIZATION:
            handleSensorStabilization();
            break;

        case STARTUP:
            handleStartup();
            break;

        case NORMAL:
            handleNormal();
            break;

        case REFILLING:
            handleRefilling();
            break;

        case DOSING_PH:
            handleDosingPH();
            break;

        case STABILIZING_PH:
            handleStabilizingPH();
            break;

        case DOSING_EC:
            handleDosingEC();
            break;

        case STABILIZING_EC:
            handleStabilizingEC();
            break;

        case SAFETY_LOCK:
            handleSafetyLock();
            break;

        default:
            break;
    }
}

//Operation Request Processing
void AutomationManager::processOperationRequest()
{
    switch(systemState.operationRequest.operation)
    {
        case OperationType::REFILL:
            processRefillOperation();
            break;

        case OperationType::PH_UP:
            processPHUpOperation();
            break;

        case OperationType::PH_DOWN:
            processPHDownOperation();
            break;


        case OperationType::RESET_SAFETY:
            processResetSafetyOperation();
            break;

        case OperationType::EC_CORRECTION:
        processECCorrectionOperation();
        break;

        default:
            break;
    }
}

//Operation Helpers

//Refill Operation
void AutomationManager::processRefillOperation()
{
    if(systemState.operationRequest.action !=
       OperationAction::START)
    {
        return;
    }

    SafetyResult result =
        safetyManager.canRefill();

    if(result != SafetyResult::SAFE)
    {
        failCurrentOperation(
            safetyManager.getSafetyReason(result));

        return;
    }

    changeState(
        REFILLING);
}

//PH Up Operation
void AutomationManager::processPHUpOperation()
{
    if(systemState.operationRequest.action !=
       OperationAction::START)
    {
        return;
    }

    SafetyResult result =
        safetyManager.canDosePH();

    if(result != SafetyResult::SAFE)
    {
        failCurrentOperation(
            safetyManager.getSafetyReason(result));

        return;
    }

    systemState.phDirection =
        PH_UP;

        systemState.correctionMode =
    CorrectionMode::MANUAL;

    systemState.firstCorrectionCycle = true;
    systemState.phAttempts = 0;
    changeState(
        DOSING_PH);
}

//PH Down Operation
void AutomationManager::processPHDownOperation()
{
    if(systemState.operationRequest.action !=
       OperationAction::START)
    {
        return;
    }

    SafetyResult result =
        safetyManager.canDosePH();

    if(result != SafetyResult::SAFE)
    {
        failCurrentOperation(
            safetyManager.getSafetyReason(result));

        return;
    }

    systemState.phDirection =
        PH_DOWN;

    systemState.correctionMode =
    CorrectionMode::MANUAL;

    systemState.firstCorrectionCycle = true;
    systemState.phAttempts = 0;

    changeState(
        DOSING_PH);
}

//reset Safety Lock Operation
void AutomationManager::processResetSafetyOperation()
{
    if(systemState.operationRequest.action !=
       OperationAction::EXECUTE && systemState.operationRequest.action != OperationAction::START)
    {
        return;
    }

    if(systemState.currentMode !=
       SAFETY_LOCK)
    {
        // Already safe or not in lock
        completeCurrentOperation();
        return;
    }

    SafetyResult result =
        safetyManager.canResetSafety();

    if(result != SafetyResult::SAFE)
    {
        failCurrentOperation(
            safetyManager.getSafetyReason(result));

        return;
    }

    // Reset is safe; clear attempt counters
    systemState.phAttempts = 0;
    systemState.ecAttempts = 0;
    systemState.safetyLock = false;

    changeState(
        STARTUP);

    completeCurrentOperation();
}

//System Validation
void AutomationManager::validateSystem()
{
    alertManager.update();

    if(alertState.lowWater)
    {
        SafetyResult result =
            safetyManager.canRefill();

        if(result != SafetyResult::SAFE)
        {
            failCurrentOperation(
                safetyManager.getSafetyReason(result));

            return;
        }

        // Create a synthetic operation request for automatic refill
        // so FirebaseManager can broadcast it to operations/current
        createOperationRequest(
            generateAutoRequestId(), // Auto-generated ID in the 32768-65535 range
            OperationType::REFILL,
            OperationAction::START,
            RequestSource::AUTOMATIC
        );

        changeState(
            REFILLING);

        return;
    }

    changeState(
        STARTUP);
}


//Operation Completion
void AutomationManager::completeCurrentOperation()
{

    if(systemState.operationRequest.state !=
       RequestState::RUNNING)
    {
        return;
    }

    systemState.operationRequest.state =
        RequestState::COMPLETED;

    systemState.operationRequest.completedTimestamp =
        millis();

    systemState.operationRequest.lastUpdatedTimestamp =
        systemState.operationRequest.completedTimestamp;

        systemState.correctionMode =
    CorrectionMode::NONE;

    systemState.firstCorrectionCycle = true;
}


//Operation Failure
void AutomationManager::failCurrentOperation(const String& reason)
{
    if(systemState.operationRequest.state !=
       RequestState::RUNNING)
    {
        return;
    }

    systemState.operationRequest.state =
        RequestState::FAILED;

    strncpy(
        systemState.operationRequest.reason,
        reason.c_str(),
        sizeof(systemState.operationRequest.reason) - 1);

    systemState.operationRequest.reason[
        sizeof(systemState.operationRequest.reason) - 1] = '\0';

    systemState.operationRequest.completedTimestamp =
        millis();

    systemState.operationRequest.lastUpdatedTimestamp =
        systemState.operationRequest.completedTimestamp;

        systemState.correctionMode =
    CorrectionMode::NONE;

    systemState.firstCorrectionCycle = true;

}

//RTC Synchronization
void AutomationManager::syncRTCFromFirebase()
{
    Serial.println(
        "RTC SYNC REQUESTED");
}

//State Change
void AutomationManager::changeState(SystemMode newMode)
{
    SystemMode oldMode =
        systemState.currentMode;

    Serial.println();
    Serial.println("================================");
    Serial.println("STATE CHANGE");
    Serial.println("================================");

    if(newMode == DOSING_PH)
    {
        if(systemState.phDirection == PH_UP)
        {
            Serial.println(
                "PH UP CORRECTION");
        }
        else
        {
            Serial.println(
                "PH DOWN CORRECTION");
        }

        Serial.print(
            "Dose Time : ");

        Serial.print(
            systemState.phDoseTime / 1000);

        Serial.println(
            " sec");
    }

    if(newMode == DOSING_EC)
    {
        Serial.println(
            "EC CORRECTION");

        Serial.print(
            "Dose Time : ");

        Serial.print(
            systemState.ecDoseTime / 1000);

        Serial.println(
            " sec");
    }

    Serial.print("FROM : ");
    Serial.println(
        getStateName(oldMode));

    Serial.print("TO   : ");
    Serial.println(
        getStateName(newMode));

    if(newMode == SAFETY_LOCK)
    {
        systemState.safetyLock = true;
        Serial.println(
            "!!! SAFETY LOCK ACTIVATED !!!");
    }

    Serial.println("================================");
    Serial.println();

    if(newMode == STARTUP)
{
    startupPhase = STARTUP_FOG_ON;

    fogCycleOn = true;
}

    systemState.currentMode =
        newMode;

    systemState.stateStartTime =
        millis();
}

//sensor stabilization
void AutomationManager::handleSensorStabilization()
{
    actuatorManager.requestCommand(FOGGER, false, "automatic", millis());

    actuatorManager.requestCommand(BLOWER, false, "automatic", millis());

    if(millis() -
       systemState.stateStartTime >=
       SENSOR_STABILIZATION_TIME)
    {
        validateSystem();
    }
}


//Startup
void AutomationManager::handleStartup()
{
    if(systemState.reservoirLocked)
    {
        return;
    }

    switch(startupPhase)
    {
        case STARTUP_FOG_ON:
        {
            actuatorManager.requestCommand(FOGGER, true, "automatic", millis());
            actuatorManager.requestCommand(BLOWER, true, "automatic", millis());

            if(millis() -
               systemState.stateStartTime >=
               STARTUP_ON_TIME)
            {
                actuatorManager.requestCommand(FOGGER, false, "automatic", millis());
                actuatorManager.requestCommand(BLOWER, false, "automatic", millis());

                startupPhase =
                    STARTUP_FOG_OFF;

                systemState.stateStartTime =
                    millis();
            }

            break;
        }

        case STARTUP_FOG_OFF:
        {
            actuatorManager.requestCommand(FOGGER, false, "automatic", millis());
            actuatorManager.requestCommand(BLOWER, false, "automatic", millis());

            if(millis() -
               systemState.stateStartTime >=
               STARTUP_OFF_TIME)
            {
                fogCycleOn = true;
                fogTimerStart = millis();

                changeState(
                    NORMAL);
            }

            break;
        }
    }
}

//Normal Operation
void AutomationManager::handleNormal()
{
    updateGrowLightSchedule();

    alertManager.update();

    validateNormalOperation();

    updateCooling();

    handleCanopyClimate();

    if (systemState.manualMode)
    {
        processFogCycle();
        return;
    }

    // Check for automatic refill before processing manual requests
    if (alertState.lowWater)
    {
        SafetyResult result = safetyManager.canRefill();
        if (result == SafetyResult::SAFE)
        {
            createOperationRequest(generateAutoRequestId(), OperationType::REFILL, OperationAction::START, RequestSource::AUTOMATIC);
            changeState(REFILLING);
            return;
        }
    }

    if(processRefillRequest())
    {
        return;
    }

    if(processPHCorrection())
    {
        return;
    }

    if(processECCorrection())
    {
        return;
    }

    processFogCycle();
}

//safety Lock Handling
bool AutomationManager::validateNormalOperation()
{
    SafetyResult result =
        safetyManager.canFog();

    if(result != SafetyResult::SAFE)
    {
        actuatorManager.requestCommand(FOGGER, false, "automatic", millis());
        actuatorManager.requestCommand(BLOWER, false, "automatic", millis());

        Serial.println(
            safetyManager.getSafetyReason(result));

        return false;
    }

    return true;
}

//Cooling Handling
void AutomationManager::updateCooling()
{
    SafetyResult result =
        safetyManager.canCool();

    if(result != SafetyResult::SAFE)
    {
        actuatorManager.requestCommand(
            PELTIER, false, "automatic", millis());

        return;
    }

    if(sensors.waterTemp >
       systemState.highWaterTemp)
    {
        actuatorManager.requestCommand(
            PELTIER, true, "automatic", millis());
    }

    if(sensors.waterTemp <
       systemState.coolerOffTemp)
    {
        actuatorManager.requestCommand(
            PELTIER, false, "automatic", millis());
    }
}

//force refill request
bool AutomationManager::processRefillRequest()
{
    if(!systemState.forceRefill)
    {
        return false;
    }

    systemState.forceRefill =
        false;

    changeState(
        REFILLING);

    return true;
}

//PH Correction Handling
bool AutomationManager::processPHCorrection()
{
    if(!alertState.phOutOfRange)
    {
        return false;
    }

    float error;

    if(sensors.ph <
       systemState.minPH)
    {
        systemState.phDirection =
            PH_UP;

        error =
            systemState.minPH -
            sensors.ph;
    }
    else
    {
        systemState.phDirection =
            PH_DOWN;

        error =
            sensors.ph -
            systemState.maxPH;
    }

    if(error < 0.3f)
    {
        systemState.phDoseTime =
            15000UL;
    }
    else if(error < 1.0f)
    {
        systemState.phDoseTime =
            30000UL;
    }
    else
    {
        systemState.phDoseTime =
            60000UL;
    }

    SafetyResult result =
        safetyManager.canDosePH();

    if(result != SafetyResult::SAFE)
    {
        failCurrentOperation(
            safetyManager.getSafetyReason(result));

        return true;
    }

    createOperationRequest(
        generateAutoRequestId(),
        systemState.phDirection == PH_UP ? OperationType::PH_UP : OperationType::PH_DOWN,
        OperationAction::START,
        RequestSource::AUTOMATIC
    );

    systemState.correctionMode =
    CorrectionMode::AUTOMATIC;

    systemState.firstCorrectionCycle = true;

    systemState.phAttempts = 0;

    changeState(
        DOSING_PH);

    return true;
}

//EC Correction Handling
bool AutomationManager::processECCorrection()
{
    if(!alertState.ecLow)
    {
        return false;
    }

    float error =
        systemState.minEC -
        sensors.ec;

    if(error < 0.2f)
    {
        systemState.ecDoseTime =
            15000UL;
    }
    else if(error < 0.5f)
    {
        systemState.ecDoseTime =
            30000UL;
    }
    else
    {
        systemState.ecDoseTime =
            60000UL;
    }

    SafetyResult result =
        safetyManager.canDoseEC();

    if(result != SafetyResult::SAFE)
    {
        failCurrentOperation(
            safetyManager.getSafetyReason(result));

        return true;
    }

    createOperationRequest(
        generateAutoRequestId(),
        OperationType::EC_CORRECTION,
        OperationAction::START,
        RequestSource::AUTOMATIC
    );

    systemState.correctionMode =
    CorrectionMode::AUTOMATIC;

    systemState.firstCorrectionCycle = true;
    systemState.ecAttempts = 0;

    changeState(
        DOSING_EC);

    return true;
}

void AutomationManager::processECCorrectionOperation()
{
    if(systemState.operationRequest.action !=
       OperationAction::START)
    {
        return;
    }

    SafetyResult result =
        safetyManager.canDoseEC();

    if(result != SafetyResult::SAFE)
    {
        failCurrentOperation(
            safetyManager.getSafetyReason(result));

        return;
    }

    systemState.correctionMode =
        CorrectionMode::MANUAL;

        systemState.firstCorrectionCycle = true;
        systemState.ecAttempts = 0;

    changeState(
        DOSING_EC);
}

//Fog Cycle Handling
void AutomationManager::processFogCycle()
{
    unsigned long elapsed =
        millis() -
        fogTimerStart;

    unsigned long fogOnTime =
        NORMAL_FOG_ON_TIME;

    unsigned long fogOffTime =
        NORMAL_FOG_OFF_TIME;

    if(sensors.temperature >
       systemState.hotFogTemperature)
    {
        fogOnTime =
            HOT_FOG_ON_TIME;

        fogOffTime =
            HOT_FOG_OFF_TIME;
    }
    else if(sensors.temperature <
            systemState.coldFogTemperature)
    {
        fogOnTime =
            COLD_FOG_ON_TIME;

        fogOffTime =
            COLD_FOG_OFF_TIME;
    }

    if(fogCycleOn)
    {
        actuatorManager.requestCommand(
            FOGGER, true, "automatic", millis());

        actuatorManager.requestCommand(
            BLOWER, true, "automatic", millis());

        if(elapsed >= fogOnTime)
        {
            fogCycleOn = false;
            fogTimerStart = millis();
        }
    }
    else
    {
        actuatorManager.requestCommand(
            FOGGER, false, "automatic", millis());

        actuatorManager.requestCommand(
            BLOWER, false, "automatic", millis());

        if(elapsed >= fogOffTime)
        {
            fogCycleOn = true;
            fogTimerStart = millis();
        }
    }
}

//Grow Light Schedule Handling
void AutomationManager::updateGrowLightSchedule()
{
    bool lightEnabled =
        isWithinSchedule(
            systemState.lightOnHour,
            systemState.lightOnMinute,
            systemState.lightOffHour,
            systemState.lightOffMinute);

    if(lightEnabled)
    {
        actuatorManager.requestCommand(
            GROW_LIGHT, true, "automatic", millis());
    }
    else
    {
        actuatorManager.requestCommand(
            GROW_LIGHT, false, "automatic", millis());
    }
}
//Get Current Time in Minutes
int AutomationManager::getCurrentMinutes() const
{
    return
        rtcManager.getHour() * 60 +
        rtcManager.getMinute();
}
//Schedule Validation
bool AutomationManager::isWithinSchedule(
    uint8_t startHour,
    uint8_t startMinute,
    uint8_t endHour,
    uint8_t endMinute) const
{
    const int current =
        getCurrentMinutes();

    const int start =
        startHour * 60 +
        startMinute;

    const int end =
        endHour * 60 +
        endMinute;

    // Normal schedule
    if(start < end)
    {
        return
            current >= start &&
            current < end;
    }

    // Overnight schedule
    return
        current >= start ||
        current < end;
}

//Refilling Handling
void AutomationManager::handleRefilling()
{
    alertManager.update();

    SafetyResult result =
        safetyManager.canRefill();

    if(result != SafetyResult::SAFE)
    {
        abortCurrentOperation(result);
        return;
    }

    systemState.reservoirLocked = true;

    actuatorManager.requestCommand(
        SOLENOID, true, "automatic", millis());

    if(sensors.waterLevel >=
       systemState.refillStopLevel)
    {
        actuatorManager.requestCommand(
            SOLENOID, false, "automatic", millis());

        systemState.reservoirLocked = false;

        completeCurrentOperation();

        changeState(
            STARTUP);

    }
}

//handle ph dosing
void AutomationManager::handleDosingPH()
{
    alertManager.update();

    SafetyResult result =
        safetyManager.canDosePH();

    if(result != SafetyResult::SAFE)
    {
        abortCurrentOperation(result);
        return;
    }

    systemState.reservoirLocked = true;

    if(systemState.phDirection == PH_UP)
    {
        actuatorManager.requestCommand(PH_UP_PUMP, true, "automatic", millis());
        actuatorManager.requestCommand(PH_DOWN_PUMP, false, "automatic", millis());
    }
    else
    {
        actuatorManager.requestCommand(PH_DOWN_PUMP, true, "automatic", millis());
        actuatorManager.requestCommand(PH_UP_PUMP, false, "automatic", millis());
    }

    bool stopDosing = false;

    if(systemState.correctionMode ==
       CorrectionMode::MANUAL &&
       systemState.firstCorrectionCycle)
    {
        if(!alertState.phOutOfRange)
        {
            stopDosing = true;
        }

        if(millis() -
           systemState.stateStartTime >=
           systemState.phDoseTime)
        {
            stopDosing = true;
        }
    }
    else
    {
        if(millis() -
           systemState.stateStartTime >=
           systemState.phDoseTime)
        {
            stopDosing = true;
        }
    }

    if(stopDosing)
    {
        actuatorManager.requestCommand(PH_UP_PUMP, false, "automatic", millis());
        actuatorManager.requestCommand(PH_DOWN_PUMP, false, "automatic", millis());

        changeState(STABILIZING_PH);
    }
}

//handle ph stabilization
void AutomationManager::handleStabilizingPH()
{
    alertManager.update();

    SafetyResult result =
        safetyManager.canDosePH();

    if(result != SafetyResult::SAFE)
    {
        abortCurrentOperation(result);
        return;
    }

    actuatorManager.requestCommand(
        PH_UP_PUMP, false, "automatic", millis());

    actuatorManager.requestCommand(
        PH_DOWN_PUMP, false, "automatic", millis());

    if(millis() -
       systemState.stateStartTime >=
       PH_STABILIZATION_TIME)
    {
        if(alertState.phOutOfRange)
        {
            systemState.phAttempts++;

            if(systemState.phAttempts >=
            MAX_PH_ATTEMPTS)
            {
                failCurrentOperation(
                    "Maximum pH correction attempts reached.");

                changeState(
                    SAFETY_LOCK);

                return;
            }

            // Recalculate error and direction to prevent wrong-way dosing if we overshot
            float error;
            if(sensors.ph < systemState.minPH) {
                systemState.phDirection = PH_UP;
                error = systemState.minPH - sensors.ph;
            } else {
                systemState.phDirection = PH_DOWN;
                error = sensors.ph - systemState.maxPH;
            }

            if(error < 0.3f) systemState.phDoseTime = 15000UL;
            else if(error < 1.0f) systemState.phDoseTime = 30000UL;
            else systemState.phDoseTime = 60000UL;

            systemState.firstCorrectionCycle = false;

            changeState(
                DOSING_PH);

            return;
        }

        systemState.phAttempts = 0;

        systemState.phDirection = PH_NONE;

        systemState.reservoirLocked = false;

        completeCurrentOperation();

        changeState(
            NORMAL);
    }
}

//handle ec dosing
void AutomationManager::handleDosingEC()
{
    alertManager.update();

    SafetyResult result =
        safetyManager.canDoseEC();

    if(result != SafetyResult::SAFE)
    {
        abortCurrentOperation(result);
        return;
    }

    systemState.reservoirLocked = true;

    actuatorManager.requestCommand(GROW_PUMP, true, "automatic", millis());
    actuatorManager.requestCommand(BLOOM_PUMP, true, "automatic", millis());

    bool stopDosing = false;

    if(systemState.correctionMode ==
       CorrectionMode::MANUAL &&
       systemState.firstCorrectionCycle)
    {
        if(!alertState.ecLow)
        {
            stopDosing = true;
        }

        if(millis() -
           systemState.stateStartTime >=
           systemState.ecDoseTime)
        {
            stopDosing = true;
        }
    }
    else
    {
        if(millis() -
           systemState.stateStartTime >=
           systemState.ecDoseTime)
        {
            stopDosing = true;
        }
    }

    if(stopDosing)
    {
        actuatorManager.requestCommand(GROW_PUMP, false, "automatic", millis());
        actuatorManager.requestCommand(BLOOM_PUMP, false, "automatic", millis());

        changeState(STABILIZING_EC);
    }
}

//handle ec stabilization
void AutomationManager::handleStabilizingEC()
{
    alertManager.update();

    SafetyResult result =
        safetyManager.canDoseEC();

    if(result != SafetyResult::SAFE)
    {
        abortCurrentOperation(result);
        return;
    }

    actuatorManager.requestCommand(
        GROW_PUMP, false, "automatic", millis());

    actuatorManager.requestCommand(
        BLOOM_PUMP, false, "automatic", millis());

    if(millis() -
       systemState.stateStartTime >=
       EC_STABILIZATION_TIME)
    {
        alertManager.update();

        if(alertState.ecLow)
        {
            systemState.ecAttempts++;

            if(systemState.ecAttempts >=
            MAX_EC_ATTEMPTS)
            {
                failCurrentOperation(
                    "Maximum EC correction attempts reached.");

                changeState(
                    SAFETY_LOCK);

                return;
            }

            systemState.firstCorrectionCycle = false;

            float error =
                systemState.minEC -
                sensors.ec;

            if(error < 0.2f)
            {
                systemState.ecDoseTime =
                    15000UL;
            }
            else if(error < 0.5f)
            {
                systemState.ecDoseTime =
                    30000UL;
            }
            else
            {
                systemState.ecDoseTime =
                    60000UL;
            }

            changeState(
                DOSING_EC);

            return;
        }

        systemState.ecAttempts = 0;

        systemState.reservoirLocked = false;

        completeCurrentOperation();

        changeState(
            NORMAL);
    }
}

//handle safety lock
void AutomationManager::handleSafetyLock()
{
    actuatorManager.turnOffAll();

    systemState.reservoirLocked = true;
}

//get state name
const char* AutomationManager::getStateName(SystemMode mode)
{
    switch(mode)
    {
        case SENSOR_STABILIZATION:
            return "SENSOR_STABILIZATION";

        case STARTUP:
            return "STARTUP";

        case NORMAL:
            return "NORMAL";

        case REFILLING:
            return "REFILLING";

        case DOSING_PH:
            return "DOSING_PH";

        case STABILIZING_PH:
            return "STABILIZING_PH";

        case DOSING_EC:
            return "DOSING_EC";

        case STABILIZING_EC:
            return "STABILIZING_EC";

        case SAFETY_LOCK:
            return "SAFETY_LOCK";

        default:
            return "UNKNOWN";
    }
}

bool AutomationManager::abortCurrentOperation(
    SafetyResult result)
{
    actuatorManager.turnOffAll();

    systemState.reservoirLocked = true;

    failCurrentOperation(
        safetyManager.getSafetyReason(result));

    changeState(
        SAFETY_LOCK);

    return true;
}

void AutomationManager::createOperationRequest(
    uint16_t requestId,
    OperationType operation,
    OperationAction action,
    RequestSource source)
{
    OperationRequest& request =
        systemState.operationRequest;

    //--------------------------------------------------
    // Identity
    //--------------------------------------------------

    request.requestId =
        requestId;

    request.operation =
        operation;

    request.action =
        action;

    request.source =
        source;

    //--------------------------------------------------
    // State
    //--------------------------------------------------

    request.state =
        RequestState::ACCEPTED;

    request.reason[0] =
        '\0';

    //--------------------------------------------------
    // Timestamps
    //--------------------------------------------------

    unsigned long now =
        millis();

    request.requestTimestamp =
        now;

    request.acceptedTimestamp =
        now;

    request.startedTimestamp =
        0;

    request.completedTimestamp =
        0;

    request.lastUpdatedTimestamp =
        now;

    //--------------------------------------------------
    // Bookkeeping
    //--------------------------------------------------

    systemState.lastProcessedRequestId =
        requestId;
}
//Canopy Climate Control
void AutomationManager::handleCanopyClimate()
{
    float temp = sensors.temperature;
    float humidity = sensors.humidity;

    uint8_t speed = 50; // Default minimum 50%

    // If sensors are failing, default to 100% for safety
    if (isnan(temp) || isnan(humidity))
    {
        speed = 100;
    }
    else
    {
        if (temp <= 22.0f)
        {
            speed = 50;
        }
        else if (temp >= 30.0f)
        {
            speed = 100;
        }
        else
        {
            // Linearly scale from 50% at 22C to 100% at 30C
            speed = 50 + (uint8_t)((temp - 22.0f) * 6.25f);
        }

        // Override for high humidity
        if (humidity > MAX_HUMIDITY)
        {
            speed = 100;
        }
    }

    actuatorManager.requestCommand(CANOPY_FAN, true, "automatic", millis(), speed);
}

// Generate a unique request ID for automatic operations
uint16_t AutomationManager::generateAutoRequestId()
{
    static uint16_t autoId = 32768;
    uint16_t id = autoId++;
    if(autoId == 0) // overflow wrapped around 65535
    {
        autoId = 32768;
    }
    return id;
}
