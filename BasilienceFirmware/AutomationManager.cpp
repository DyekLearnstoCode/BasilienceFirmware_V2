#include "AutomationManager.h"

#include "Config.h"
#include "Globals.h"

namespace
{
    // Local to this file so it does not collide with FirebaseManager.cpp's
    // own (internal-linkage) operationToString() used for RTDB payloads.
    const char* latencyOperationName(OperationType operation)
    {
        switch(operation)
        {
            case OperationType::REFILL: return "REFILL";
            case OperationType::PH_UP: return "PH_UP";
            case OperationType::PH_DOWN: return "PH_DOWN";
            case OperationType::EC_CORRECTION: return "EC_CORRECTION";
            default: return "NONE";
        }
    }
}

void AutomationManager::begin()
{
    startupPhase = STARTUP_FOG_ON;

    fogCycleOn = true;
    activeFogStrategy = "";

    systemState.currentMode =
        SENSOR_STABILIZATION;

    systemState.stateStartTime =
        millis();

    fogTimerStart = millis();
}

void AutomationManager::update()
{
    // Support actuators are reconciled for every FSM state, including operation
    // requests that return early below. Both physical and mock inputs have already
    // been normalized into the same effective sensors structure before this call.
    updateCooling();

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
        if(systemState.operationRequest.startedTimestamp != 0 &&
           millis() - systemState.operationRequest.startedTimestamp >= OPERATION_TIMEOUT_MS)
        {
            abortCurrentOperation("Operation timeout");
            return;
        }

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
    // State Machine
    //--------------------------------------------------

    processCurrentState();

} //Core Framework

void AutomationManager::setManualCoolingDemand(bool active)
{
    manualCoolingDemandActive = active;
}

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

    if(systemState.currentMode != REFILLING)
    {
        changeState(REFILLING);
        return;
    }

    // RUNNING operations are routed here before processCurrentState(). Continue
    // the active state handler so the stop threshold can complete the request.
    handleRefilling();
}

//PH Up Operation
void AutomationManager::processPHUpOperation()
{
    if(systemState.operationRequest.action !=
       OperationAction::START)
    {
        return;
    }

    if(systemState.currentMode == DOSING_PH)
    {
        handleDosingPH();
        return;
    }

    if(systemState.currentMode == STABILIZING_PH)
    {
        handleStabilizingPH();
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

    {
        const float error = systemState.phTargetMin - sensors.ph;
        systemState.phDoseTime = error < 0.3f ? 5000UL : (error < 1.0f ? 10000UL : 15000UL);
    }

    systemState.correctionMode =
        systemState.operationRequest.source == RequestSource::AUTOMATIC ?
            CorrectionMode::AUTOMATIC : CorrectionMode::MANUAL;

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

    if(systemState.currentMode == DOSING_PH)
    {
        handleDosingPH();
        return;
    }

    if(systemState.currentMode == STABILIZING_PH)
    {
        handleStabilizingPH();
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

    {
        const float error = sensors.ph - systemState.phTargetMax;
        systemState.phDoseTime = error < 0.3f ? 5000UL : (error < 1.0f ? 10000UL : 15000UL);
    }

    systemState.correctionMode =
        systemState.operationRequest.source == RequestSource::AUTOMATIC ?
            CorrectionMode::AUTOMATIC : CorrectionMode::MANUAL;

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

    String reason;
    if(!safetyManager.resetRecoverableSubsystems(reason))
    {
        failCurrentOperation(reason.isEmpty() ? "No locked subsystem can be reset." : reason);
        return;
    }

    systemState.phAttempts = 0;
    systemState.ecAttempts = 0;
    systemState.reservoirLocked = false;
    if(systemState.currentMode == SAFETY_LOCK) changeState(STARTUP);
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

    if (systemState.operationRequest.operation == OperationType::PH_UP ||
        systemState.operationRequest.operation == OperationType::PH_DOWN ||
        systemState.operationRequest.operation == OperationType::EC_CORRECTION ||
        systemState.operationRequest.operation == OperationType::REFILL)
    {
        Serial.print("[LATENCY] localComplete t=");
        Serial.print(systemState.operationRequest.completedTimestamp);
        Serial.print(" requestId=");
        Serial.print(systemState.operationRequest.requestId);
        Serial.print(" operation=");
        Serial.println(latencyOperationName(systemState.operationRequest.operation));
    }

    if (systemState.operationRequest.source == RequestSource::AUTOMATIC &&
        (systemState.operationRequest.operation == OperationType::PH_UP ||
         systemState.operationRequest.operation == OperationType::PH_DOWN ||
         systemState.operationRequest.operation == OperationType::EC_CORRECTION))
    {
        systemState.chemistryFoggingHoldActive = true;
        systemState.chemistryFoggingHoldStartTime =
            systemState.operationRequest.completedTimestamp;

        Serial.println("[CHEMISTRY] Correction complete - waiting for lifecycle publication");
    }

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

    if(newMode == oldMode)
    {
        return;
    }

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
    activeFogStrategy = "";
}

    if(newMode == REFILLING)
    {
        refillDiagnosticsInitialized = false;
    }

    if(newMode == DOSING_PH || newMode == STABILIZING_PH ||
       newMode == DOSING_EC || newMode == STABILIZING_EC)
    {
        suspendAutomaticRootFogging("Chemistry correction active");
    }

    systemState.currentMode =
        newMode;

    systemState.stateStartTime =
        millis();
}

//sensor stabilization
void AutomationManager::handleSensorStabilization()
{
    suspendAutomaticRootFogging("Waiting for valid startup sensor readings");

    // Stabilization no longer gates every reservoir-control subsystem. Each
    // path's safety check requires the readings it needs to be finite and valid.
    // Applied mock data is therefore authoritative on the next local pass.
    if(processReadyLocalRegulation())
    {
        return;
    }

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

    // Retain the startup fog cycle without making it an exclusive 120-second
    // gate in front of ready local reservoir-control inputs.
    if(processReadyLocalRegulation())
    {
        return;
    }

    switch(startupPhase)
    {
        case STARTUP_FOG_ON:
        {
            actuatorManager.requestCommand(FOGGER, true, "automatic", millis(), 100, "startup");
            actuatorManager.requestCommand(BLOWER, true, "automatic", millis(), 100, "startup");

            if(millis() -
               systemState.stateStartTime >=
               STARTUP_ON_TIME)
            {
                actuatorManager.requestCommand(FOGGER, false, "automatic", millis(), 100, "startup");
                actuatorManager.requestCommand(BLOWER, false, "automatic", millis(), 100, "startup");

                startupPhase =
                    STARTUP_FOG_OFF;

                systemState.stateStartTime =
                    millis();
            }

            break;
        }

        case STARTUP_FOG_OFF:
        {
            actuatorManager.requestCommand(FOGGER, false, "automatic", millis(), 100, "startup");
            actuatorManager.requestCommand(BLOWER, false, "automatic", millis(), 100, "startup");

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

    const bool fogCycleAllowed = validateNormalOperation();

    handleCanopyClimate();

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

    if(fogCycleAllowed)
    {
        processFogCycle();
    }
}

bool AutomationManager::processReadyLocalRegulation()
{
    alertManager.update();

    if(alertState.lowWater)
    {
        SafetyResult result = safetyManager.canRefill();
        if(result == SafetyResult::SAFE)
        {
            createOperationRequest(
                generateAutoRequestId(),
                OperationType::REFILL,
                OperationAction::START,
                RequestSource::AUTOMATIC);
            changeState(REFILLING);
            return true;
        }
    }

    if(processRefillRequest()) return true;
    if(processPHCorrection()) return true;
    if(processECCorrection()) return true;

    return false;
}

void AutomationManager::suspendAutomaticRootFogging(const String& reason)
{
    actuatorManager.requestCommand(
        FOGGER, false, "automatic", millis(), 100, "", reason);
    actuatorManager.requestCommand(
        BLOWER, false, "automatic", millis(), 100, "", reason);
}

//safety Lock Handling
bool AutomationManager::validateNormalOperation()
{
    static bool diagnosticInitialized = false;
    static SafetyResult lastDiagnosticResult = SafetyResult::SAFE;
    static bool coolingFoggingSuspendedLogged = false;

    SafetyResult result =
        safetyManager.canFog();

    if(result != SafetyResult::SAFE)
    {
        actuatorManager.requestCommand(FOGGER, false, "automatic", millis());
        actuatorManager.requestCommand(BLOWER, false, "automatic", millis());

        if (!diagnosticInitialized || result != lastDiagnosticResult)
        {
            Serial.print("[SAFETY] ");
            Serial.println(safetyManager.getSafetyReason(result));
        }

        diagnosticInitialized = true;
        lastDiagnosticResult = result;

        return false;
    }

    if (diagnosticInitialized && lastDiagnosticResult != SafetyResult::SAFE)
    {
        Serial.println("[SAFETY] Normal operation restored");
    }

    diagnosticInitialized = true;
    lastDiagnosticResult = SafetyResult::SAFE;

    // Root fogging must stay off for the entire duration active water
    // cooling is required, independent of the chemistry hold below.
    // coolingDemandActive is the same authoritative signal updateCooling()
    // itself uses to drive circulation/Peltier - not a separate condition.
    if (coolingDemandActive)
    {
        actuatorManager.requestCommand(FOGGER, false, "automatic", millis());
        actuatorManager.requestCommand(BLOWER, false, "automatic", millis());

        if (!coolingFoggingSuspendedLogged)
        {
            Serial.println("[COOLING] Root fogging suspended during water cooling");
            coolingFoggingSuspendedLogged = true;
        }

        return false;
    }

    if (coolingFoggingSuspendedLogged)
    {
        Serial.println("[COOLING] Root fogging eligible after cooling");
        coolingFoggingSuspendedLogged = false;
    }

    // canFog() already confirmed pH/EC are in range and no dosing/stabilizing
    // is active. A just-completed automatic chemistry correction still holds
    // Fogger/Blower back until FirebaseManager confirms the COMPLETED write,
    // unless the bounded local grace period has elapsed - Firebase
    // availability must never be a plant-survival dependency.
    if (systemState.chemistryFoggingHoldActive)
    {
        if (millis() - systemState.chemistryFoggingHoldStartTime >=
            CHEMISTRY_FOGGING_HOLD_TIMEOUT_MS)
        {
            systemState.chemistryFoggingHoldActive = false;
            Serial.println("[CHEMISTRY] Cloud unavailable - releasing fogging from local safe state");
        }
        else
        {
            return false;
        }
    }

    return true;
}

// Cooling and reservoir circulation handling
void AutomationManager::updateCooling()
{
    const SafetyResult coolingSafety = safetyManager.canCool();

    const int8_t temperatureBand =
        sensors.waterTemp > systemState.highWaterTemp ? 1 :
        (sensors.waterTemp < systemState.coolerOffTemp ? -1 : 0);

    if (coolingSafety != SafetyResult::SAFE)
    {
        coolingDemandActive = false;
        manualCoolingDemandActive = false;
    }
    else if (temperatureBand == 1)
    {
        coolingDemandActive = true;
    }
    else if (temperatureBand == -1)
    {
        coolingDemandActive = false;
        manualCoolingDemandActive = false;
    }

    if (temperatureBand != lastWaterTemperatureBand)
    {
        Serial.print("[TEMP] water="); Serial.print(sensors.waterTemp, 2);
        Serial.print(" high="); Serial.print(systemState.highWaterTemp, 2);
        Serial.print(" coolerOff="); Serial.println(systemState.coolerOffTemp, 2);
        if (temperatureBand == 1 && coolingSafety == SafetyResult::SAFE)
            Serial.println("[TEMP] Peltier requested");
        else if (temperatureBand == -1 || coolingSafety != SafetyResult::SAFE)
            Serial.println("[TEMP] Peltier OFF requested");
        lastWaterTemperatureBand = temperatureBand;
    }

    const bool phStabilizationActive =
        systemState.currentMode == STABILIZING_PH;
    const bool ecStabilizationActive =
        systemState.currentMode == STABILIZING_EC;

    constexpr uint8_t DEMAND_PELTIER = 0x01;
    constexpr uint8_t DEMAND_PH = 0x02;
    constexpr uint8_t DEMAND_EC = 0x04;

    uint8_t demandMask = 0;
    if (coolingDemandActive || manualCoolingDemandActive) demandMask |= DEMAND_PELTIER;
    if (phStabilizationActive) demandMask |= DEMAND_PH;
    if (ecStabilizationActive) demandMask |= DEMAND_EC;

    if (!circulationDiagnosticsInitialized || demandMask != lastCirculationDemandMask)
    {
        const uint8_t added = demandMask & ~lastCirculationDemandMask;
        const uint8_t removed = lastCirculationDemandMask & ~demandMask;

        if (added & DEMAND_PELTIER) Serial.println("[CIRCULATION] Demand added: PELTIER");
        if (added & DEMAND_PH) Serial.println("[CIRCULATION] Demand added: PH_STABILIZATION");
        if (added & DEMAND_EC) Serial.println("[CIRCULATION] Demand added: EC_STABILIZATION");
        if (removed & DEMAND_PELTIER) Serial.println("[CIRCULATION] Demand removed: PELTIER");
        if (removed & DEMAND_PH) Serial.println("[CIRCULATION] Demand removed: PH_STABILIZATION");
        if (removed & DEMAND_EC) Serial.println("[CIRCULATION] Demand removed: EC_STABILIZATION");

        if (removed != 0 && demandMask != 0)
        {
            Serial.print("[CIRCULATION] Remaining demand: ");
            Serial.println(getCirculationReason(demandMask));
        }

        lastCirculationDemandMask = demandMask;
        circulationDiagnosticsInitialized = true;
    }

    const String circulationReason = getCirculationReason(demandMask);
    actuatorManager.requestCommand(
        CIRCULATION_PUMP,
        demandMask != 0,
        "automatic",
        millis(),
        100,
        "",
        circulationReason);

    const ActuatorStatus circulationStatus =
        actuatorManager.getStatus(CIRCULATION_PUMP);
    const bool circulationConfirmed =
        circulationStatus.running &&
        circulationStatus.state == ActuatorCommandState::RUNNING;

    if (circulationConfirmed != lastCirculationRunning)
    {
        Serial.println(circulationConfirmed
            ? "[CIRCULATION] Pump ON"
            : "[CIRCULATION] Pump OFF");
        lastCirculationRunning = circulationConfirmed;
    }

    if (circulationStatus.state != lastCirculationState)
    {
        if (circulationStatus.state == ActuatorCommandState::REJECTED)
        {
            Serial.print("[CIRCULATION] REJECTED: ");
            Serial.println(circulationStatus.reason);
        }
        lastCirculationState = circulationStatus.state;
    }

    if (coolingDemandActive && circulationConfirmed)
    {
        actuatorManager.requestCommand(
            PELTIER, true, "automatic", millis());
    }
    else
    {
        actuatorManager.requestCommand(
            PELTIER,
            false,
            "automatic",
            millis(),
            100,
            "",
            coolingDemandActive ? "waiting_for_circulation" : "");
    }

    const bool peltierRunning = actuatorManager.getStatus(PELTIER).running;
    if (peltierRunning != lastPeltierRunning)
    {
        if (peltierRunning)
        {
            Serial.println("[TEMP] Circulation confirmed");
            Serial.println("[TEMP] Peltier RUNNING");
        }
        else
        {
            Serial.println("[TEMP] Peltier OFF");
        }
        lastPeltierRunning = peltierRunning;
    }
}

String AutomationManager::getCirculationReason(uint8_t demandMask) const
{
    String reason;
    if (demandMask & 0x01) reason = "temperature_circulation";
    if (demandMask & 0x02)
    {
        if (!reason.isEmpty()) reason += "+";
        reason += "ph_stabilization";
    }
    if (demandMask & 0x04)
    {
        if (!reason.isEmpty()) reason += "+";
        reason += "ec_stabilization";
    }
    return reason;
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
    if(systemState.phSubsystemLocked)
        return false;

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

        // minPH/maxPH remain the acceptable-range trigger only; dose size is
        // computed against the correction target, matching every retry dose.
        error =
            systemState.phTargetMin -
            sensors.ph;
    }
    else
    {
        systemState.phDirection =
            PH_DOWN;

        error =
            sensors.ph -
            systemState.phTargetMax;
    }

    if(error < 0.3f)
    {
        systemState.phDoseTime =
            5000UL;
    }
    else if(error < 1.0f)
    {
        systemState.phDoseTime =
            10000UL;
    }
    else
    {
        systemState.phDoseTime =
            15000UL;
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

    Serial.print("[PH] value="); Serial.print(sensors.ph, 2);
    Serial.print(" min="); Serial.print(systemState.minPH, 2);
    Serial.print(" max="); Serial.println(systemState.maxPH, 2);
    Serial.print("[PH] requesting ");
    Serial.println(systemState.phDirection == PH_UP ? "PH_UP" : "PH_DOWN");

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
    if(systemState.ecSubsystemLocked)
        return false;

    if(!alertState.ecLow && !alertState.ecHigh)
    {
        return false;
    }

    systemState.ecDirection = alertState.ecLow ? EC_RAISE : EC_DILUTE;

    float error = systemState.ecDirection == EC_RAISE
        ? systemState.ecTargetMin - sensors.ec
        : sensors.ec - systemState.ecTargetMax;

    if(error < 0.2f)
    {
        systemState.ecDoseTime =
            5000UL;
    }
    else if(error < 0.5f)
    {
        systemState.ecDoseTime =
            10000UL;
    }
    else
    {
        systemState.ecDoseTime =
            15000UL;
    }

    SafetyResult result = systemState.ecDirection == EC_RAISE
        ? safetyManager.canDoseEC()
        : safetyManager.canDiluteEC();

    if(result != SafetyResult::SAFE)
    {
        if(systemState.ecDirection == EC_DILUTE && result == SafetyResult::RESERVOIR_FULL)
        {
            systemState.ecSubsystemLocked = true;
            actuatorManager.requestCommand(GROW_PUMP, false, "automatic", millis());
            actuatorManager.requestCommand(BLOOM_PUMP, false, "automatic", millis());
            actuatorManager.requestCommand(SOLENOID, false, "automatic", millis(), 100,
                "dilution", "Reservoir full; manual EC attention required");
            Serial.println("[EC] Dilution blocked: reservoir full; manual attention required");
        }
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

    Serial.print("[EC] value="); Serial.print(sensors.ec, 2);
    Serial.print(" min="); Serial.print(systemState.minEC, 2);
    Serial.print(" max="); Serial.println(systemState.maxEC, 2);
    Serial.println(systemState.ecDirection == EC_RAISE
        ? "[EC] requesting nutrient correction"
        : "[EC] requesting dilution correction");

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

    if(systemState.currentMode == DOSING_EC)
    {
        handleDosingEC();
        return;
    }

    if(systemState.currentMode == STABILIZING_EC)
    {
        handleStabilizingEC();
        return;
    }

    systemState.ecDirection = sensors.ec < systemState.minEC ? EC_RAISE : EC_DILUTE;
    const float error = systemState.ecDirection == EC_RAISE
        ? systemState.ecTargetMin - sensors.ec
        : sensors.ec - systemState.ecTargetMax;
    systemState.ecDoseTime = error < 0.2f ? 5000UL : (error < 0.5f ? 10000UL : 15000UL);

    SafetyResult result = systemState.ecDirection == EC_RAISE
        ? safetyManager.canDoseEC()
        : safetyManager.canDiluteEC();

    if(result != SafetyResult::SAFE)
    {
        if(systemState.ecDirection == EC_DILUTE && result == SafetyResult::RESERVOIR_FULL)
        {
            systemState.ecSubsystemLocked = true;
            actuatorManager.requestCommand(GROW_PUMP, false, "automatic", millis());
            actuatorManager.requestCommand(BLOOM_PUMP, false, "automatic", millis());
            actuatorManager.requestCommand(SOLENOID, false, "automatic", millis(), 100,
                "dilution", "Reservoir full; manual EC attention required");
            Serial.println("[EC] Dilution blocked: reservoir full; manual attention required");
        }
        failCurrentOperation(
            safetyManager.getSafetyReason(result));

        return;
    }

    systemState.correctionMode =
        systemState.operationRequest.source == RequestSource::AUTOMATIC ?
            CorrectionMode::AUTOMATIC : CorrectionMode::MANUAL;

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

    String fogStrategy = "normal";

    if(sensors.temperature >
       systemState.hotFogTemperature)
    {
        fogStrategy =
            "hot";
    }
    else if(sensors.temperature <
            systemState.coldFogTemperature)
    {
        fogStrategy =
            "cold";
    }

    if(activeFogStrategy == "")
    {
        activeFogStrategy =
            fogStrategy;
    }

    unsigned long fogOnTime =
        NORMAL_FOG_ON_TIME;

    unsigned long fogOffTime =
        NORMAL_FOG_OFF_TIME;

    if(activeFogStrategy == "hot")
    {
        fogOnTime =
            HOT_FOG_ON_TIME;

        fogOffTime =
            HOT_FOG_OFF_TIME;
    }
    else if(activeFogStrategy == "cold")
    {
        fogOnTime =
            COLD_FOG_ON_TIME;

        fogOffTime =
            COLD_FOG_OFF_TIME;
    }

    if(fogCycleOn)
    {
        actuatorManager.requestCommand(
            FOGGER, true, "automatic", millis(), 100, activeFogStrategy);

        actuatorManager.requestCommand(
            BLOWER, true, "automatic", millis(), 100, activeFogStrategy);

        if(elapsed >= fogOnTime)
        {
            fogCycleOn = false;
            fogTimerStart = millis();
        }
    }
    else
    {
        actuatorManager.requestCommand(
            FOGGER, false, "automatic", millis(), 100, activeFogStrategy);

        actuatorManager.requestCommand(
            BLOWER, false, "automatic", millis(), 100, activeFogStrategy);

        if(elapsed >= fogOffTime)
        {
            fogCycleOn = true;
            fogTimerStart = millis();
            activeFogStrategy = "";
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

    const bool mockSource = systemState.mockSensorsEnabled;
    const bool diagnosticsChanged =
        !refillDiagnosticsInitialized ||
        mockSource != lastRefillMockSource ||
        fabsf(sensors.waterLevel - lastRefillWaterLevel) > 0.01f ||
        fabsf(systemState.refillStartLevel - lastRefillStartLevel) > 0.01f ||
        fabsf(systemState.refillStopLevel - lastRefillStopLevel) > 0.01f;

    if(diagnosticsChanged)
    {
        Serial.print("[REFILL] source=");
        Serial.println(mockSource ? "MOCK" : "PHYSICAL");
        Serial.print("[REFILL] waterLevel=");
        Serial.println(sensors.waterLevel, 2);
        Serial.print("[REFILL] refillStartLevel=");
        Serial.println(systemState.refillStartLevel, 2);
        Serial.print("[REFILL] refillStopLevel=");
        Serial.println(systemState.refillStopLevel, 2);

        refillDiagnosticsInitialized = true;
        lastRefillMockSource = mockSource;
        lastRefillWaterLevel = sensors.waterLevel;
        lastRefillStartLevel = systemState.refillStartLevel;
        lastRefillStopLevel = systemState.refillStopLevel;
    }

    if(sensors.waterLevel >=
       systemState.refillStopLevel)
    {
        Serial.println("[REFILL] Stop threshold reached");

        actuatorManager.requestCommand(
            SOLENOID, false, "automatic", millis());

        Serial.println("[REFILL] Solenoid OFF");

        systemState.reservoirLocked = false;

        completeCurrentOperation();

        Serial.println("[REFILL] Operation COMPLETED");

        changeState(
            STARTUP);

        return;
    }

    actuatorManager.requestCommand(
        SOLENOID, true, "automatic", millis());
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
        const bool targetReached =
            systemState.phDirection == PH_UP
                ? sensors.ph >= systemState.phTargetMin
                : sensors.ph <= systemState.phTargetMax;

        if(!targetReached)
        {
            systemState.phAttempts++;

            if(systemState.phAttempts >=
            MAX_PH_ATTEMPTS)
            {
                failCurrentSubsystem("Maximum pH correction attempts reached.");

                return;
            }

            // Continue toward the inner target. Only reverse direction after an
            // actual overshoot beyond the opposite inner target.
            if(systemState.phDirection == PH_UP && sensors.ph > systemState.phTargetMax)
                systemState.phDirection = PH_DOWN;
            else if(systemState.phDirection == PH_DOWN && sensors.ph < systemState.phTargetMin)
                systemState.phDirection = PH_UP;

            const float error = systemState.phDirection == PH_UP
                ? systemState.phTargetMin - sensors.ph
                : sensors.ph - systemState.phTargetMax;

            if(error < 0.3f) systemState.phDoseTime = 5000UL;
            else if(error < 1.0f) systemState.phDoseTime = 10000UL;
            else systemState.phDoseTime = 15000UL;

            systemState.firstCorrectionCycle = false;

            changeState(
                DOSING_PH);

            return;
        }

        systemState.phAttempts = 0;

        systemState.phDirection = PH_NONE;

        systemState.reservoirLocked = false;

        completeCurrentOperation();

        Serial.println("[PH] correction completed");

        changeState(
            NORMAL);
    }
}

//handle ec dosing
void AutomationManager::handleDosingEC()
{
    alertManager.update();

    SafetyResult result = systemState.ecDirection == EC_RAISE
        ? safetyManager.canDoseEC()
        : safetyManager.canDiluteEC();

    if(result != SafetyResult::SAFE)
    {
        abortCurrentOperation(result);
        return;
    }

    systemState.reservoirLocked = true;

    if(systemState.ecDirection == EC_RAISE)
    {
        actuatorManager.requestCommand(SOLENOID, false, "automatic", millis());
        actuatorManager.requestCommand(GROW_PUMP, true, "automatic", millis());
        actuatorManager.requestCommand(BLOOM_PUMP, true, "automatic", millis());
    }
    else
    {
        actuatorManager.requestCommand(GROW_PUMP, false, "automatic", millis());
        actuatorManager.requestCommand(BLOOM_PUMP, false, "automatic", millis());
        actuatorManager.requestCommand(SOLENOID, true, "automatic", millis(), 100,
            "dilution", "ec_high_dilution");
    }

    bool stopDosing = false;

    if(systemState.correctionMode ==
       CorrectionMode::MANUAL &&
       systemState.firstCorrectionCycle)
    {
        const bool targetReached = systemState.ecDirection == EC_RAISE
            ? sensors.ec >= systemState.ecTargetMin
            : sensors.ec <= systemState.ecTargetMax;
        if(targetReached)
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
        actuatorManager.requestCommand(SOLENOID, false, "automatic", millis(), 100,
            systemState.ecDirection == EC_DILUTE ? "dilution" : "",
            systemState.ecDirection == EC_DILUTE ? "dilution_interval_complete" : "");

        changeState(STABILIZING_EC);
    }
}

//handle ec stabilization
void AutomationManager::handleStabilizingEC()
{
    alertManager.update();

    SafetyResult result = systemState.ecDirection == EC_RAISE
        ? safetyManager.canDoseEC()
        : safetyManager.canDiluteEC();

    // A full reservoir is a terminal dilution condition only while EC still
    // requires correction. The solenoid must never continue adding water.
    if(result == SafetyResult::RESERVOIR_FULL && systemState.ecDirection == EC_DILUTE)
    {
        failCurrentSubsystem("Reservoir reached refill stop level before EC target; manual attention required.");
        return;
    }

    if(result != SafetyResult::SAFE)
    {
        abortCurrentOperation(result);
        return;
    }

    actuatorManager.requestCommand(
        GROW_PUMP, false, "automatic", millis());

    actuatorManager.requestCommand(
        BLOOM_PUMP, false, "automatic", millis());

    actuatorManager.requestCommand(
        SOLENOID, false, "automatic", millis(), 100,
        systemState.ecDirection == EC_DILUTE ? "dilution" : "");

    if(millis() -
       systemState.stateStartTime >=
       EC_STABILIZATION_TIME)
    {
        alertManager.update();

        const bool targetReached =
            systemState.ecDirection == EC_RAISE
                ? sensors.ec >= systemState.ecTargetMin
                : sensors.ec <= systemState.ecTargetMax;

        if(!targetReached)
        {
            systemState.ecAttempts++;

            if(systemState.ecAttempts >=
            MAX_EC_ATTEMPTS)
            {
                failCurrentSubsystem("Maximum EC correction attempts reached; manual attention required.");

                return;
            }

            systemState.firstCorrectionCycle = false;

            float error = systemState.ecDirection == EC_RAISE
                ? systemState.ecTargetMin - sensors.ec
                : sensors.ec - systemState.ecTargetMax;

            if(error < 0.2f)
            {
                systemState.ecDoseTime =
                    5000UL;
            }
            else if(error < 0.5f)
            {
                systemState.ecDoseTime =
                    10000UL;
            }
            else
            {
                systemState.ecDoseTime =
                    15000UL;
            }

            changeState(
                DOSING_EC);

            return;
        }

        systemState.ecAttempts = 0;

        systemState.ecDirection = EC_NONE;

        systemState.reservoirLocked = false;

        completeCurrentOperation();

        Serial.println("[EC] correction completed");

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
    return abortCurrentOperation(safetyManager.getSafetyReason(result));
}

bool AutomationManager::abortCurrentOperation(
    const String& reason)
{
    Serial.print("[SAFETY] ");
    Serial.print(getStateName(systemState.currentMode));
    Serial.print(" stopped: ");
    Serial.println(reason);

    failCurrentSubsystem(reason);

    return true;
}

void AutomationManager::failCurrentSubsystem(const String& reason)
{
    const OperationType operation = systemState.operationRequest.operation;

    if(operation == OperationType::PH_UP || operation == OperationType::PH_DOWN ||
       systemState.currentMode == DOSING_PH || systemState.currentMode == STABILIZING_PH)
    {
        suspendAutomaticRootFogging("pH remains outside the acceptable range");
        actuatorManager.requestCommand(PH_UP_PUMP, false, "automatic", millis(), 100, "", reason);
        actuatorManager.requestCommand(PH_DOWN_PUMP, false, "automatic", millis(), 100, "", reason);
        systemState.phSubsystemLocked = true;
    }
    else if(operation == OperationType::EC_CORRECTION ||
            systemState.currentMode == DOSING_EC || systemState.currentMode == STABILIZING_EC)
    {
        suspendAutomaticRootFogging("EC remains outside the acceptable range");
        actuatorManager.requestCommand(GROW_PUMP, false, "automatic", millis(), 100, "", reason);
        actuatorManager.requestCommand(BLOOM_PUMP, false, "automatic", millis(), 100, "", reason);
        actuatorManager.requestCommand(SOLENOID, false, "automatic", millis(), 100,
            systemState.ecDirection == EC_DILUTE ? "dilution" : "", reason);
        systemState.ecSubsystemLocked = true;
        systemState.ecDirection = EC_NONE;
    }
    else if(operation == OperationType::REFILL || systemState.currentMode == REFILLING)
    {
        actuatorManager.requestCommand(SOLENOID, false, "automatic", millis(), 100, "refill", reason);
        systemState.refillSubsystemLocked = true;
    }
    else
    {
        // The global mechanism remains available for a genuinely system-wide
        // critical condition, but ordinary subsystem failures never reach it.
        actuatorManager.turnOffAll(reason);
        systemState.safetyLock = true;
        changeState(SAFETY_LOCK);
        failCurrentOperation(reason);
        return;
    }

    systemState.reservoirLocked = false;
    failCurrentOperation(reason);
    changeState(NORMAL);
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

    uint8_t speed = 50;

    // DHT failure is isolated from reservoir chemistry. The environmental
    // fail-safe keeps ventilation at full speed.
    if (isnan(temp) || isnan(humidity))
    {
        speed = 100;
    }
    else
    {
        if (!highAirDemandActive && temp > systemState.highAirTemp)
            highAirDemandActive = true;
        else if (highAirDemandActive && temp <= systemState.airTempRelease)
            highAirDemandActive = false;

        if (!highHumidityDemandActive && humidity > systemState.highHumidity)
            highHumidityDemandActive = true;
        else if (highHumidityDemandActive && humidity <= systemState.humidityRelease)
            highHumidityDemandActive = false;

        if (highAirDemandActive || highHumidityDemandActive) speed = 100;
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
