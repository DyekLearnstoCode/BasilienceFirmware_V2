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

    // Serial Monitor Focus Mode compact input summaries (see
    // DebugManager::shouldPrintDebug()'s own comment) - "meaningfully
    // changed" for a float that is only ever reassigned (never blended)
    // when a genuinely new accepted value lands, e.g. sensors.ph/ec/
    // waterLevelCm - see applyEffectiveSensors()'s own comment. NaN<->NaN
    // is "unchanged" (both mean "no accepted value"); anything else where
    // either side is NaN, or the two differ at all, is "changed".
    bool inputValueChanged(float a, float b)
    {
        if (isnan(a) && isnan(b)) return false;
        if (isnan(a) != isnan(b)) return true;
        return a != b;
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

bool AutomationManager::automationAllowed(AutomationTestSubsystem subsystem) const
{
    return systemState.automationTestSubsystem == AutomationTestSubsystem::NONE ||
        systemState.automationTestSubsystem == subsystem;
}

AutomationTestSubsystem AutomationManager::subsystemForOperation(OperationType operation) const
{
    switch(operation)
    {
        case OperationType::REFILL: return AutomationTestSubsystem::REFILL;
        case OperationType::PH_UP:
        case OperationType::PH_DOWN: return AutomationTestSubsystem::PH;
        case OperationType::EC_CORRECTION: return AutomationTestSubsystem::EC;
        default: return AutomationTestSubsystem::NONE;
    }
}

void AutomationManager::reconcileAutomationTestMode()
{
    const AutomationTestSubsystem selected = systemState.automationTestSubsystem;

    if(!automationTestModeInitialized)
    {
        automationTestModeInitialized = true;
        lastAutomationTestSubsystem = selected;
        if(selected == AutomationTestSubsystem::NONE) return;
    }
    else if(selected == lastAutomationTestSubsystem)
    {
        return;
    }

    lastAutomationTestSubsystem = selected;

    Serial.print("[AUTO-TEST] mode=");
    Serial.println(automationTestSubsystemName(selected));

    cancelPausedAutomaticOperation();
    stopPausedAutomaticControllers();

    if(selected == AutomationTestSubsystem::NONE)
    {
        Serial.println("[AUTO-TEST] full automation restored");
        return;
    }

    Serial.print("[AUTO-TEST] ");
    Serial.print(automationTestSubsystemName(selected));
    Serial.println(" controller enabled; unrelated controllers paused");

    if(selected == AutomationTestSubsystem::STARTUP)
    {
        startupPhase = STARTUP_FOG_ON;
        fogCycleOn = true;
        activeFogStrategy = "";
        fogTimerStart = millis();

        if(systemState.currentMode == SENSOR_STABILIZATION)
            systemState.stateStartTime = millis();
        else
            changeState(SENSOR_STABILIZATION);
    }
    else if(systemState.currentMode != SAFETY_LOCK)
    {
        // Every non-startup isolated controller enters through handleNormal(),
        // which contains the one centralized set of controller entry gates -
        // UNLESS a currently in-flight AUTOMATIC operation's owner remains
        // allowed under the new selection: the exact same condition
        // cancelPausedAutomaticOperation() above just used to decide NOT to
        // cancel it. Forcing NORMAL here regardless of that would silently
        // abandon the operation mid-flight (e.g. mid DOSING_PH/
        // STABILIZING_PH) without ever releasing reservoirLocked/
        // phDirection/phAttempts - those stay exactly as an in-progress
        // correction left them, landing in NORMAL, which then permanently
        // blocks canDosePH()'s reservoir-lock check (reservoirLocked true,
        // currentMode no longer DOSING_PH/STABILIZING_PH so the check no
        // longer recognizes it as this correction's own) for every future
        // correction attempt. Let a still-allowed operation reach its own
        // defined end state instead, matching stopCultivationChemistry()'s
        // own stated assumption that stabilization is "short,
        // self-terminating" - true only if nothing else yanks currentMode
        // out from under it first.
        const OperationRequest& activeRequest = systemState.operationRequest;
        const bool hasPreservedAutomaticOperation =
            activeRequest.source == RequestSource::AUTOMATIC &&
            (activeRequest.state == RequestState::ACCEPTED ||
             activeRequest.state == RequestState::RUNNING) &&
            automationAllowed(subsystemForOperation(activeRequest.operation));

        if(!hasPreservedAutomaticOperation)
        {
            changeState(NORMAL);
        }
    }

    // Isolated FOGGING test entry needs the same fresh-cycle reset STARTUP
    // already gives itself above. Without this, fogCycleOn/fogTimerStart/
    // activeFogStrategy are whatever Full System's own in-progress fog
    // cycle last left them - most commonly fogCycleOn=true with a
    // fogTimerStart from well over fogOnTime ago, so the very next
    // processFogCycle() call (a) commands FOGGER/BLOWER ON for one tick
    // simply because fogCycleOn was still true, then (b) immediately finds
    // elapsed >= fogOnTime against that stale timer and flips to OFF - a
    // sub-second transient pulse, not a real cycle. Resetting here instead
    // gives the isolated test the same deterministic "fresh ON phase, full
    // configured duration" start as a real boot (see
    // AutomationManager::begin()) or a STARTUP-mode entry, and the normal
    // 5-minute scheduler in processFogCycle() itself is untouched.
    if(selected == AutomationTestSubsystem::FOGGING)
    {
        fogCycleOn = true;
        activeFogStrategy = "";
        fogTimerStart = millis();
    }

    // Isolated EC test entry rearm (targeted EC_SUBSYSTEM_LOCKED fix): a
    // PREVIOUS EC episode that exhausted MAX_EC_ATTEMPTS, or hit a
    // since-possibly-resolved RESERVOIR_FULL dilution block (see
    // failCurrentSubsystem()/processECCorrection()'s EC branches), leaves
    // ecSubsystemLocked latched with only two existing ways back: the
    // NORMAL-automation recovery rearm further down in update() (only fires
    // once sensors.ec is back inside [minEC, maxEC] - exactly the condition
    // the prior attempt was failing to reach in the first place) or an
    // admin's explicit Reset Safety. Neither fires just from starting a new
    // EC test session, so a stale lock from an old completed/failed test
    // could otherwise block every future EC test indefinitely even with a
    // healthy, stable EC reading (the observed EC_SUBSYSTEM_LOCKED report).
    // This branch runs EXACTLY ONCE per genuine mode transition into EC
    // (gated by the selected==lastAutomationTestSubsystem dedupe at the top
    // of this function, not every tick), so it cannot be used to repeatedly
    // reopen the attempt budget and enable unbounded dosing - MAX_EC_ATTEMPTS
    // still applies in full to the fresh session this rearms into. The
    // global hard safety lock is deliberately left untouched: if
    // systemState.safetyLock is active, the rearm is skipped entirely, and
    // every per-attempt safety check (canDoseEC()/canDiluteEC(): water
    // level, reservoir ownership/full-dilution, sensor validity) still
    // re-runs unchanged on the very next correction attempt regardless -
    // this only grants a fresh attempt budget, it never bypasses a
    // currently real condition.
    if(selected == AutomationTestSubsystem::EC && systemState.ecSubsystemLocked)
    {
        if(systemState.safetyLock)
        {
            Serial.println("[EC-LOCK] rearm skipped reason=hard_safety_lock_active");
        }
        else
        {
            systemState.ecSubsystemLocked = false;
            systemState.ecAttempts = 0;
            systemState.ecDirection = EC_NONE;
            Serial.println("[EC-LOCK] cleared reason=test_session_rearm");
            Serial.println("[EC-LOCK] test-session rearm");
        }
    }

    // Starting a new isolated test is an explicit request for a fresh bounded
    // attempt budget. Clear only the selected subsystem's stale prior-test
    // latch; the global hard safety lock and every live sensor/water safety
    // check remain authoritative on the next tick. Like the EC rearm above,
    // this runs once per genuine mode transition, so it cannot defeat the
    // per-session MAX_PH_ATTEMPTS/MAX_REFILL_ATTEMPTS limits.
    if(selected == AutomationTestSubsystem::PH && systemState.phSubsystemLocked)
    {
        if(systemState.safetyLock)
        {
            Serial.println("[PH-LOCK] rearm skipped reason=hard_safety_lock_active");
        }
        else
        {
            systemState.phSubsystemLocked = false;
            systemState.phAttempts = 0;
            systemState.phDirection = PH_NONE;
            Serial.println("[PH-LOCK] cleared reason=test_session_rearm");
        }
    }

    if(selected == AutomationTestSubsystem::REFILL && systemState.refillSubsystemLocked)
    {
        if(systemState.safetyLock)
        {
            Serial.println("[REFILL-LOCK] rearm skipped reason=hard_safety_lock_active");
        }
        else
        {
            systemState.refillSubsystemLocked = false;
            resetAutomaticRefillAttempts();
            Serial.println("[REFILL-LOCK] cleared reason=test_session_rearm");
        }
    }
}

void AutomationManager::cancelPausedAutomaticOperation()
{
    OperationRequest& request = systemState.operationRequest;
    if(request.source != RequestSource::AUTOMATIC ||
       (request.state != RequestState::ACCEPTED && request.state != RequestState::RUNNING))
    {
        return;
    }

    const AutomationTestSubsystem owner = subsystemForOperation(request.operation);
    if(owner == AutomationTestSubsystem::NONE || automationAllowed(owner)) return;

    const String reason = "Paused by Automation Test Mode";
    actuatorManager.requestCommand(SOLENOID, false, "automatic", millis(), 100, "", reason);
    actuatorManager.requestCommand(GROW_PUMP, false, "automatic", millis(), 100, "", reason);
    actuatorManager.requestCommand(BLOOM_PUMP, false, "automatic", millis(), 100, "", reason);
    actuatorManager.requestCommand(PH_UP_PUMP, false, "automatic", millis(), 100, "", reason);
    actuatorManager.requestCommand(PH_DOWN_PUMP, false, "automatic", millis(), 100, "", reason);

    if(request.state == RequestState::RUNNING)
    {
        failCurrentOperation(reason);
    }
    else
    {
        request.state = RequestState::FAILED;
        strncpy(request.reason, reason.c_str(), sizeof(request.reason) - 1);
        request.reason[sizeof(request.reason) - 1] = '\0';
        request.completedTimestamp = millis();
        request.lastUpdatedTimestamp = request.completedTimestamp;
    }

    systemState.reservoirLocked = false;
    systemState.phDirection = PH_NONE;
    systemState.ecDirection = EC_NONE;
    systemState.phAttempts = 0;
    systemState.ecAttempts = 0;
}

void AutomationManager::stopPausedAutomaticControllers()
{
    const OperationRequest& request = systemState.operationRequest;
    const AutomationTestSubsystem manualOperationOwner =
        request.source == RequestSource::MANUAL &&
        (request.state == RequestState::ACCEPTED || request.state == RequestState::RUNNING)
            ? subsystemForOperation(request.operation)
            : AutomationTestSubsystem::NONE;

    const auto allowedOrManual = [&](AutomationTestSubsystem subsystem)
    {
        return automationAllowed(subsystem) || manualOperationOwner == subsystem;
    };

    if(!allowedOrManual(AutomationTestSubsystem::STARTUP) &&
       !allowedOrManual(AutomationTestSubsystem::FOGGING))
    {
        suspendAutomaticRootFogging("Paused by Automation Test Mode");
    }
    if(!allowedOrManual(AutomationTestSubsystem::GROW_LIGHT))
        actuatorManager.requestCommand(GROW_LIGHT, false, "automatic", millis(), 100, "", "Paused by Automation Test Mode");
    if(!allowedOrManual(AutomationTestSubsystem::CANOPY))
        actuatorManager.requestCommand(CANOPY_FAN, false, "automatic", millis(), 100, "", "Paused by Automation Test Mode");
    if(!allowedOrManual(AutomationTestSubsystem::PH))
    {
        actuatorManager.requestCommand(PH_UP_PUMP, false, "automatic", millis(), 100, "", "Paused by Automation Test Mode");
        actuatorManager.requestCommand(PH_DOWN_PUMP, false, "automatic", millis(), 100, "", "Paused by Automation Test Mode");
    }
    if(!allowedOrManual(AutomationTestSubsystem::EC))
    {
        actuatorManager.requestCommand(GROW_PUMP, false, "automatic", millis(), 100, "", "Paused by Automation Test Mode");
        actuatorManager.requestCommand(BLOOM_PUMP, false, "automatic", millis(), 100, "", "Paused by Automation Test Mode");
    }
    if(!allowedOrManual(AutomationTestSubsystem::REFILL) &&
       !allowedOrManual(AutomationTestSubsystem::EC))
        actuatorManager.requestCommand(SOLENOID, false, "automatic", millis(), 100, "", "Paused by Automation Test Mode");
    if(!allowedOrManual(AutomationTestSubsystem::COOLING))
        actuatorManager.requestCommand(PELTIER, false, "automatic", millis(), 100, "", "Paused by Automation Test Mode");
    if(!allowedOrManual(AutomationTestSubsystem::COOLING) &&
       !allowedOrManual(AutomationTestSubsystem::PH) &&
       !allowedOrManual(AutomationTestSubsystem::EC))
        actuatorManager.requestCommand(CIRCULATION_PUMP, false, "automatic", millis(), 100, "", "Paused by Automation Test Mode");
}

void AutomationManager::update()
{
    reconcileAutomationTestMode();

    // Support actuators are reconciled for every FSM state, including operation
    // requests that return early below. Both physical and mock inputs have already
    // been normalized into the same effective sensors structure before this call.
    //
    // Deliberately ahead of the cultivation gate below - CONFIRMED, updated
    // scope: water-TEMPERATURE SENSING and ALERTS (AlertManager::
    // updateWaterTemperatureAlert(), called from within updateCooling()
    // unconditionally) and MANUAL Peltier control must run regardless of
    // cultivation state, so this call site stays here, ahead of the gate.
    // AUTOMATIC cooling itself, however, now DOES require an active
    // cultivation cycle (or isolated Cooling Automation Test Mode for bench
    // testing) - see updateCooling()'s own comment on automaticCoolingAllowed
    // for the full reasoning and how ending a cycle mid-cooling safely
    // returns the pulse state machine to IDLE. The previous version of this
    // comment claimed cooling "must run whether or not a growth cycle
    // exists" without qualification - that was accurate for the old
    // single-gate design and is no longer accurate for automatic cooling.
    updateCooling();

    // Manual root fogging (processManualFogPairing()) must likewise run
    // regardless of cultivation state, OperationRequest lifecycle, or RTC
    // sync below - manual root-fogging is testable at any time, matching
    // every other manual actuator - see item 11 of the confirmed Manual
    // Mode audit.
    processManualFogPairing();

    // Observes the active-cycle flag and acts on its transitions. Also runs
    // ahead of the operation lifecycle below so a chemistry dose that must not
    // continue is stopped before its state handler can command a pump.
    updateCultivationGate();

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
    // Cultivation gate
    //
    // Placed after the always-run duties above and before any cultivation
    // state processing. Gating processCurrentState() as a whole - rather than
    // handleNormal() alone - is what stops SENSOR_STABILIZATION/STARTUP's own
    // timer-driven progression (and the STARTUP fog sequence) from advancing
    // while no growth cycle is active.
    //--------------------------------------------------

    if(systemState.automationTestSubsystem == AutomationTestSubsystem::NONE &&
       !harvestScheduleCache.isActive())
    {
        handleCultivationPaused();
        return;
    }

    //--------------------------------------------------
    // State Machine
    //--------------------------------------------------

    processCurrentState();

} //Core Framework

//==================================================
// Cultivation cycle gate
//==================================================

void AutomationManager::updateCultivationGate()
{
    const bool active = harvestScheduleCache.isActive();

    if(!cultivationStateInitialized)
    {
        // First evaluation after boot. The flag came from NVS, so this is
        // equally valid with no network at all.
        cultivationStateInitialized = true;
        cultivationActive = active;

        if(active)
        {
            Serial.print("[CYCLE] Restored active cycle from NVS: ");
            Serial.print(harvestScheduleCache.getCycleId());
            Serial.print(" (#");
            Serial.print(harvestScheduleCache.getCycleNumber());
            Serial.println(")");
            Serial.println("[AUTOMATION] Cultivation enabled (offline-capable)");
        }
        else
        {
            Serial.println("[CYCLE] No persisted active cycle");
            Serial.println("[AUTOMATION] Cultivation paused - no active growth cycle");
        }
    }
    else if(active != cultivationActive)
    {
        cultivationActive = active;

        if(active)
        {
            Serial.print("[CYCLE] Active cycle: ");
            Serial.print(harvestScheduleCache.getCycleId());
            Serial.print(" (#");
            Serial.print(harvestScheduleCache.getCycleNumber());
            Serial.println(")");
            Serial.println("[AUTOMATION] Cultivation enabled");

            // Re-enter through stabilization rather than jumping to NORMAL:
            // it re-validates the current readings, lets validateSystem()
            // divert to REFILLING if the reservoir is already low, and stops a
            // stale reading from triggering an immediate dose.
            startupPhase = STARTUP_FOG_ON;
            fogCycleOn = true;
            activeFogStrategy = "";
            fogTimerStart = millis();
            changeState(SENSOR_STABILIZATION);
        }
        else
        {
            Serial.println("[CYCLE] Cycle completed or inactive");
            Serial.println("[AUTOMATION] Cultivation paused");

            // Fog timers must not keep counting as though a cycle were still
            // running; a later activation starts them fresh.
            fogCycleOn = true;
            activeFogStrategy = "";
        }
    }

    if(!active)
    {
        // Runs every iteration while paused, not only on the transition, so a
        // stabilization retry cannot slip a fresh dose through afterwards.
        stopCultivationChemistry();
    }
}

// Stops chemistry that must not continue once the cycle is inactive.
//
// Deliberately does NOT use abortCurrentOperation(): that routes through
// failCurrentSubsystem(), which latches phSubsystemLocked/ecSubsystemLocked and
// would leave the device needing an admin safety reset after an ordinary cycle
// completion. failCurrentOperation() closes the request without touching any
// subsystem lock.
//
// Stabilization and an in-flight refill are intentionally absent: they are
// short, self-terminating, and reach a defined end state on their own.
void AutomationManager::stopCultivationChemistry()
{
    const SystemMode mode = systemState.currentMode;

    if(mode != DOSING_PH && mode != DOSING_EC)
    {
        return;
    }

    const String reason = "Growth cycle is no longer active";

    if(mode == DOSING_PH)
    {
        actuatorManager.requestCommand(PH_UP_PUMP, false, "automatic", millis(), 100, "", reason);
        actuatorManager.requestCommand(PH_DOWN_PUMP, false, "automatic", millis(), 100, "", reason);
        systemState.phDirection = PH_NONE;
        systemState.phAttempts = 0;
        Serial.println("[CYCLE] pH dosing stopped - cycle no longer active");
    }
    else
    {
        actuatorManager.requestCommand(GROW_PUMP, false, "automatic", millis(), 100, "", reason);
        actuatorManager.requestCommand(BLOOM_PUMP, false, "automatic", millis(), 100, "", reason);
        systemState.ecDirection = EC_NONE;
        systemState.ecAttempts = 0;
        Serial.println("[CYCLE] Nutrient dosing stopped - cycle no longer active");
    }

    failCurrentOperation(reason);

    // The reservoir was held for this correction only; releasing it here stops
    // completion from leaving the lock stuck true.
    systemState.reservoirLocked = false;

    changeState(NORMAL);
}

// Idle reconciliation while no growth cycle is active.
//
// Every command below is an OFF for an actuator that is already off in the
// steady state, and ActuatorManager::requestCommand() discards a repeated OFF
// for something not running - so this does not write every loop. It also never
// overrides a manual command: requestCommand() ignores automatic requests for
// an actuator the user has manually taken in Manual Mode.
void AutomationManager::handleCultivationPaused()
{
    suspendAutomaticRootFogging("No active growth cycle");

    actuatorManager.requestCommand(GROW_LIGHT, false, "automatic", millis());

    actuatorManager.requestCommand(PH_UP_PUMP, false, "automatic", millis());
    actuatorManager.requestCommand(PH_DOWN_PUMP, false, "automatic", millis());
    actuatorManager.requestCommand(GROW_PUMP, false, "automatic", millis());
    actuatorManager.requestCommand(BLOOM_PUMP, false, "automatic", millis());

    // An active refill routes through the operation lifecycle and returns
    // before this handler, so reaching here means no refill is in progress.
    actuatorManager.requestCommand(SOLENOID, false, "automatic", millis());

    // CONFIRMED BUG FIX (no-active-cultivation-cycle behavior): the canopy
    // fan used to be kept running at a safe 70%/last-automatic-speed
    // baseline here instead of stopping, on the reasoning that ventilation
    // is a general good regardless of cultivation state. Per the confirmed
    // production rule, "no active cultivation cycle" means NO automatic
    // cultivation actuator operates, full stop - the canopy fan is an
    // automatic cultivation actuator like any other in this function
    // (grow light, dosing pumps, solenoid, fogging/blower above), not a
    // standing exception. Commanded OFF every tick this handler runs (the
    // same pattern already used for every other actuator above), so a fan
    // left running automatically when the cycle ends is commanded off on
    // the very next control update once cultivation goes inactive.
    // Unaffected: manual control (this is still an "automatic"-source
    // command, so an admin's manual ON under Manual Mode still outranks it
    // exactly as it already does for every other actuator here - see
    // ActuatorManager::requestCommand()'s manual-hold priority), and
    // isolated Automation Test Mode (this function is never reached at all
    // while any subsystem is isolated - see the cultivation gate in
    // update()), so bench testing is unaffected either way.
    actuatorManager.requestCommand(CANOPY_FAN, false, "automatic", millis());
}

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

    if(!canStartNewPHCorrection())
    {
        failCurrentOperation(
            "pH reading is not currently stable; retry once a fresh stable reading is confirmed.");

        return;
    }

    systemState.phDirection =
        PH_UP;

    systemState.phDoseTime = PH_DOSING_TIME;

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

    if(!canStartNewPHCorrection())
    {
        failCurrentOperation(
            "pH reading is not currently stable; retry once a fresh stable reading is confirmed.");

        return;
    }

    systemState.phDirection =
        PH_DOWN;

    systemState.phDoseTime = PH_DOSING_TIME;

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

    // SENSOR_STABILIZATION remains a pure initialization/wait phase for
    // ordinary regulation: pH/EC correction still belongs solely to NORMAL,
    // once the full STARTUP fog sequence has completed (see the automation
    // case-matrix resolution, cases 2/3). The one exception is the
    // pre-startup low-water check below - plants should not begin
    // acclimating under the STARTUP fog sequence with an unsafe water level,
    // so this is the narrowest possible reintroduction of that single
    // decision, not a return to the old processReadyLocalRegulation() path.
    //
    // automationAllowed(REFILL) keeps isolated Automation Test Mode (STARTUP
    // or any other single subsystem) from silently invoking REFILL
    // automation - an isolated STARTUP test must stay isolated, per the
    // automation case-matrix. ignoreWaterLevelAutomation is the existing
    // developer bypass: real water measurement/status stays active, but the
    // pre-startup refill requirement is skipped so the bench can proceed
    // straight to STARTUP.
    // autoRefillEligible() (reservoir/refill lifecycle audit fix) - CONFIRMED
    // BUG FIX: this site previously used alertState.lowWater && shouldAutoRefill()
    // directly, omitting sensors.refillStartConfirmed entirely - the only one
    // of the two production automatic-refill trigger sites that did. A
    // pre-STARTUP refill could therefore begin on the debounced alert flag
    // alone, without the same trusted HC-SR04 reading-agreement confirmation
    // handleNormal()'s own trigger already required. Both sites now share
    // the exact same eligibility condition - see its own comment in
    // AutomationManager.h.
    if (automationAllowed(AutomationTestSubsystem::REFILL) &&
        autoRefillEligible() &&
        !systemState.ignoreWaterLevelAutomation)
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

    // Serial Monitor Focus Mode (see DebugManager's own comments) - purely
    // whether this transition prints, never whether it happens. Every state
    // mutation below (safetyLock, startupPhase, etc.) stays unconditional.
    const bool printTransition =
        debugManager.shouldPrintStateTransition(oldMode, newMode);

    if(printTransition)
    {
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
    }

    if(newMode == SAFETY_LOCK)
    {
        systemState.safetyLock = true;
        if(printTransition)
        {
            Serial.println(
                "!!! SAFETY LOCK ACTIVATED !!!");
        }
    }

    if(printTransition)
    {
        Serial.println("================================");
        Serial.println();
    }

if(newMode == STARTUP)
{
    startupPhase = STARTUP_FOG_ON;

    fogCycleOn = true;
    activeFogStrategy = "";
}

    if(newMode == REFILLING)
    {
        refillDiagnosticsInitialized = false;
        resetAutomaticRefillAttempts();
    }

    if(newMode == STABILIZING_PH)
    {
        // See the member's own comment: unset until updateCooling() confirms
        // circulation is actually running for this fresh episode.
        phStabilizationCirculationConfirmedAt = 0;

        // Quiet-monitoring/4-minute-budget redesign: each dose cycle within
        // a correction gets its own checkpoint/trend bookkeeping, reset here
        // (not per whole-correction) so a redose still gets a fresh 90s
        // checkpoint pulse and a fresh trend baseline - only
        // correctionCycleStartAt itself (Types.h) persists across redoses.
        systemState.phFirstCheckpointPublished = false;
        systemState.phStableSince = 0;
        systemState.phStableCheckpointPublished = false;
        systemState.phTrendReferenceValue = NAN;
        systemState.phLastTrendCheckAt = 0;

        // Off on entry - handleStabilizingPH() sets it true itself, fresh
        // every tick, once past the initial settle window.
        systemState.phWatchPhaseActive = false;
    }

    if(newMode == STABILIZING_EC)
    {
        // See ecStabilizationCirculationConfirmedAt's own comment.
        ecStabilizationCirculationConfirmedAt = 0;

        // Mirrors STABILIZING_PH's own reset above.
        systemState.ecFirstCheckpointPublished = false;
        systemState.ecStableSince = 0;
        systemState.ecStableCheckpointPublished = false;
        systemState.ecTrendReferenceValue = NAN;
        systemState.ecLastTrendCheckAt = 0;
        systemState.ecWatchPhaseActive = false;
    }

    // A redose (STABILIZING_PH/EC -> DOSING_PH/EC) leaves the watch flag
    // stale-true otherwise - the pump is about to run, fogging must not be
    // allowed to think it is still in the safe watch phase.
    if(newMode == DOSING_PH)
    {
        systemState.phWatchPhaseActive = false;
    }

    if(newMode == DOSING_EC)
    {
        systemState.ecWatchPhaseActive = false;
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

    // SENSOR_STABILIZATION is a true initialization/wait phase: ordinary
    // automatic refill/pH/EC regulation must not pre-empt it (see the
    // automation case-matrix resolution, cases 2/3). This state now waits
    // for pH and EC to actually confirm a stable reading (the same
    // isPhCurrentlyStable()/isEcCurrentlyStable() gate canStartNewPHCorrection()/
    // canStartNewECCorrection() trust elsewhere) rather than a blind timer -
    // a fixed wait could either hand off to STARTUP before
    // PH_EC_ANALOG_SETTLE_TIME's analog charge-up window has even finished
    // (leaving pH/EC published as NaN into STARTUP), or needlessly hold the
    // system in this state after readings are already good. In mock mode
    // there is no analog settle to wait out, so mock sensors are treated as
    // immediately ready. SENSOR_STABILIZATION_TIME remains as a hard cap so
    // a genuinely stuck/disconnected probe still reaches STARTUP - where
    // SafetyManager's own validPH()/validEC() gates continue to hold dosing
    // off - instead of hanging here indefinitely.
    const bool sensorsReady =
        systemState.mockSensorsEnabled ||
        (sensorManager.isPhCurrentlyStable() && sensorManager.isEcCurrentlyStable());

    if(sensorsReady ||
       millis() - systemState.stateStartTime >= SENSOR_STABILIZATION_TIME)
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

    // Ordinary automatic refill/pH/EC regulation must not pre-empt the
    // startup fog sequence either - it only begins once NORMAL is reached
    // (see the automation case-matrix resolution, cases 2/3).

    // Serial Monitor Focus Mode: startup timer/progress and purge-phase
    // diagnostics, additive (no such periodic print previously existed) -
    // see DebugManager::shouldPrintDebug()'s own comment. Throttled to
    // AUTO_TEST_BLOCK_LOG_INTERVAL_MS, same cadence as the existing
    // isolated-mode diagnostics elsewhere in this file.
    const bool dbgStartup = debugManager.shouldPrintDebug(DebugCategory::STARTUP);
    static unsigned long lastStartupProgressLogAt = 0;
    const unsigned long nowForStartupLog = millis();
    const bool startupProgressDue = dbgStartup &&
        (lastStartupProgressLogAt == 0 ||
         nowForStartupLog - lastStartupProgressLogAt >= AUTO_TEST_BLOCK_LOG_INTERVAL_MS);

    switch(startupPhase)
    {
        case STARTUP_FOG_ON:
        {
            actuatorManager.requestCommand(FOGGER, true, "automatic", millis(), 100, "startup");

            // Startup runs the Blower at a fixed 100%, not the
            // lastAutomaticCanopySpeed climate tiering NORMAL mode uses -
            // startup is a one-time initial fog/airflow push, so it
            // intentionally always runs at full speed regardless of the
            // current air-temp/humidity demand. NORMAL mode (70/100/50%
            // hysteresis - see handleCanopyClimate()) only takes over once
            // startup finishes.
            actuatorManager.requestCommand(BLOWER, true, "automatic", millis(), 100, "startup");

            if (startupProgressDue)
            {
                lastStartupProgressLogAt = nowForStartupLog;
                Serial.print("[STARTUP] FOG_ON elapsed=");
                Serial.print((millis() - systemState.stateStartTime) / 1000UL);
                Serial.print("s / ");
                Serial.print(STARTUP_ON_TIME / 1000UL);
                Serial.println("s");
            }

            if(millis() -
               systemState.stateStartTime >=
               STARTUP_ON_TIME)
            {
                actuatorManager.requestCommand(FOGGER, false, "automatic", millis(), 100, "startup");
                // Blower is deliberately left commanded on here - the
                // STARTUP_FOG_OFF case below takes over on the very next
                // tick and keeps it on for BLOWER_PURGE_MS using the same
                // stateStartTime reset just below, so it is never actually
                // turned off and immediately back on.

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

            unsigned long elapsedInOff =
                millis() - systemState.stateStartTime;

            // Blower purge: stays on for the first BLOWER_PURGE_MS of the
            // startup rest phase to clear fog concentrated near the
            // reservoir toward the root chamber, then off for the
            // remainder of the unchanged STARTUP_OFF_TIME window. The fogger
            // is already off for the whole purge window by design, so the
            // blower's automatic fogger-running gate is waived here (see
            // ActuatorManager::validateCommand's BLOWER case).
            //
            // Deliberately fixed at 100%, NOT systemState.blowerSpeedPercent:
            // this is a purge/clearing phase, not the configured normal
            // fogging airflow, so it intentionally always runs at full speed
            // regardless of the configured automatic fogging percentage.
            {
                // The existing full system retains its blower purge. The
                // isolated Startup contract tests Fogger+Blower as an exact
                // pair, so both remain OFF for the complete 60-second phase.
                bool purging =
                    systemState.automationTestSubsystem != AutomationTestSubsystem::STARTUP &&
                    elapsedInOff < BLOWER_PURGE_MS;

                if (startupProgressDue)
                {
                    lastStartupProgressLogAt = nowForStartupLog;
                    Serial.print("[STARTUP] FOG_OFF elapsed=");
                    Serial.print(elapsedInOff / 1000UL);
                    Serial.print("s / ");
                    Serial.print(STARTUP_OFF_TIME / 1000UL);
                    Serial.print("s purge=");
                    Serial.println(purging ? "ON" : "OFF");
                }

                actuatorManager.requestCommand(
                    BLOWER, purging, "automatic", millis(), 100, "startup", "", false, purging);
            }

            if(elapsedInOff >=
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
    if(automationAllowed(AutomationTestSubsystem::GROW_LIGHT))
    {
        updateGrowLightSchedule();
    }

    alertManager.update();

    // Diagnostics only - see the header's own comment. No-ops unless
    // isolated PH or EC Automation Test Mode is currently selected.
    logAutomationTestBlockReason();

    const bool fogControllerAllowed =
        automationAllowed(AutomationTestSubsystem::FOGGING);
    const bool fogCycleAllowed =
        fogControllerAllowed && validateNormalOperation();

    if(automationAllowed(AutomationTestSubsystem::CANOPY))
    {
        handleCanopyClimate();
    }

    // Automatic re-arm for a max-attempt bounded-refill failure lock. The
    // only place that ever sets refillSubsystemLocked for a REFILL operation
    // is handleBoundedAutomaticRefill()'s MAX_REFILL_ATTEMPTS exhaustion (via
    // failCurrentSubsystem()) - it means the last bounded-refill episode
    // never reached refillStopLevelCm, not that the reservoir is permanently
    // unusable. Once the water level itself genuinely recovers to/above the
    // runtime stop level, the condition the lock was raised for is gone, so
    // clear it here - event-driven off the real reading, never a timer - so
    // a fresh low-water episode later is free to run its own full 3-attempt
    // cycle. An admin's explicit Reset Safety (resetRecoverableSubsystems(),
    // a weaker "sensor is valid" bar) remains available as before for
    // clearing it without waiting on the water itself.
    //
    // refillStopConfirmed (not a plain sensors.waterLevelCm >=
    // refillStopLevelCm comparison) - see SensorManager::readWaterLevel().
    // The plain comparison let a single accepted reading at/above the stop
    // level clear the lock immediately, before the same 3-consecutive-
    // accepted-reading confirmation every other stop-threshold consumer
    // (handleRefilling()'s own completion check) already requires.
    if (systemState.refillSubsystemLocked &&
        sensors.refillStopConfirmed)
    {
        systemState.refillSubsystemLocked = false;
        if (debugManager.shouldPrintDebug(DebugCategory::WATER))
        {
            Serial.println("[REFILL] refill subsystem lock cleared - water level recovered to stop level");
            Serial.print("[REFILL-LOCK] CLEARED reason=water_recovered depth=");
            Serial.println(sensors.waterLevelCm, 2);
            Serial.print("[REFILL-LOCK] CLEAR-WRITER source=water_recovery_rearm depth=");
            Serial.println(sensors.waterLevelCm, 2);
        }
    }

    // Diagnostics only, no behavior: report the lock/auto-refill-eligibility
    // state whenever the low-water alert is active, but only on a change to
    // locked/shouldAutoRefill so this cannot spam every loop while water sits
    // low and unchanged between test runs.
    if (alertState.lowWater && debugManager.shouldPrintDebug(DebugCategory::WATER))
    {
        static bool checkLogInitialized = false;
        static bool lastLoggedLocked = false;
        static bool lastLoggedShouldAutoRefill = false;

        const bool shouldRefillNow = shouldAutoRefill();

        if (!checkLogInitialized ||
            systemState.refillSubsystemLocked != lastLoggedLocked ||
            shouldRefillNow != lastLoggedShouldAutoRefill)
        {
            checkLogInitialized = true;
            lastLoggedLocked = systemState.refillSubsystemLocked;
            lastLoggedShouldAutoRefill = shouldRefillNow;

            Serial.print("[REFILL-LOCK] CHECK locked=");
            Serial.print(systemState.refillSubsystemLocked ? "true" : "false");
            Serial.print(" depth=");
            Serial.print(sensors.waterLevelCm, 2);
            Serial.print(" start=");
            Serial.print(systemState.refillStartLevelCm, 2);
            Serial.print(" stop=");
            Serial.print(systemState.refillStopLevelCm, 2);
            Serial.print(" shouldAutoRefill=");
            Serial.println(shouldRefillNow ? "true" : "false");
        }
    }

    // Check for automatic refill. Manual "Trigger Refill" requests never
    // reach this function at all - they arrive as an OperationRequest
    // (OperationType::REFILL, RequestSource::MANUAL) and are dispatched by
    // the operation-lifecycle block earlier in updateAutomation(), before
    // handleNormal() is ever called for that tick - see
    // processRefillOperation().
    // refillSubsystemLocked is checked explicitly here, not only inside
    // canRefill() below, so a persistent low-water condition that already
    // exhausted MAX_REFILL_ATTEMPTS can never start a brand-new bounded
    // refill operation (a fresh "starting attempt 1") while that same
    // unresolved episode continues - only the re-arm check above, or an
    // explicit admin reset, can lift it.
    // autoRefillEligible() (reservoir/refill lifecycle audit fix) is the
    // single shared eligibility condition - see its own comment in
    // AutomationManager.h for why sensors.refillStartConfirmed (the TRUSTED
    // HC-SR04 confirmation) is required on top of the debounced
    // alertState.lowWater flag, and why every automatic-refill trigger site
    // (this one and validateSystem()'s pre-STARTUP one) must use the exact
    // same condition rather than each re-deriving its own.
    if (automationAllowed(AutomationTestSubsystem::REFILL) &&
        !systemState.refillSubsystemLocked &&
        autoRefillEligible() &&
        !systemState.ignoreWaterLevelAutomation)
    {
        SafetyResult result = safetyManager.canRefill();
        if (result == SafetyResult::SAFE)
        {
            if (debugManager.shouldPrintDebug(DebugCategory::WATER))
            {
                Serial.print("[REFILL] depth=");
                Serial.print(sensors.waterLevelCm, 2);
                Serial.print("cm startThreshold=");
                Serial.print(systemState.refillStartLevelCm, 2);
                Serial.println("cm -> START");
                Serial.print("[REFILL-LOCK] START allowed locked=");
                Serial.print(systemState.refillSubsystemLocked ? "true" : "false");
                Serial.print(" depth=");
                Serial.println(sensors.waterLevelCm, 2);
            }

            createOperationRequest(generateAutoRequestId(), OperationType::REFILL, OperationAction::START, RequestSource::AUTOMATIC);
            changeState(REFILLING);
            return;
        }
    }

    // Automatic re-arm for a terminal PH failure latch, mirroring the
    // refillSubsystemLocked re-arm above: phSubsystemLocked only ever
    // reflects the PH_EC_CORRECTION_STALL_TIMEOUT_MS budget expiring on a
    // genuinely stalled reading for the LAST out-of-range episode (set in
    // failCurrentSubsystem()'s PH branch), not that pH is permanently
    // unsafe. Once pH itself genuinely recovers inside [minPH, maxPH], the
    // condition the lock was raised for is gone, so clear it here -
    // event-driven off the real reading, never a timer - together with
    // phAttempts, so a later, independent out-of-range episode (in either
    // direction) gets its own fresh budget from correctionCycleStartAt = 0.
    // Deliberately NOT the mere fact that phDirection would
    // flip - e.g. 4.50 failing then jumping straight to 7.00 is still out
    // of range on both bounds and must stay locked; only an actual reading
    // inside both minPH and maxPH counts. phDirection is left at PH_NONE
    // (already set by failCurrentSubsystem() on terminal failure) since no
    // correction is running to need a direction. An admin's explicit Reset
    // Safety (resetRecoverableSubsystems(), a weaker "sensor is valid" bar)
    // remains available as before for clearing it without waiting on pH
    // itself.
    //
    // sensors.ph is the stability WINDOW's last-accepted representative
    // value (StabilityWindow::lastStable), retained on display/publication
    // even while the live incoming signal is currently unstable and hasn't
    // reconfirmed it - see updateStabilityWindow()'s own comment. An
    // in-range retained value alone is therefore not sufficient evidence pH
    // is genuinely safe right now; canStartNewPHCorrection() already treats
    // sensorManager.isPhCurrentlyStable() as the authoritative "is this
    // reading current" signal for starting a correction, and this re-arm
    // reuses the same one rather than inventing a second stability check.
    // Mock mode is the one case that needs a different source: it bypasses
    // the physical stability window entirely (applyEffectiveSensors()
    // assigns sensors = systemState.mockSensors directly and never feeds
    // updateStabilityWindow()), so phStabilityWindow.currentlyStable simply
    // stays at whatever it last was under physical sourcing - not a
    // reflection of the current mock value - and gating on it here would
    // risk permanently blocking re-arm under mock. sensors.ph is already
    // this tick's live mock value, so using it directly is correct and
    // matches how mock has always been consumed elsewhere.
    const bool phReadingIsCurrent =
        systemState.mockSensorsEnabled || sensorManager.isPhCurrentlyStable();

    if (systemState.phSubsystemLocked &&
        isfinite(sensors.ph) &&
        sensors.ph >= systemState.minPH &&
        sensors.ph <= systemState.maxPH &&
        phReadingIsCurrent)
    {
        systemState.phSubsystemLocked = false;
        systemState.phAttempts = 0;
        systemState.phDirection = PH_NONE;
        Serial.println("[PH] pH subsystem lock cleared - pH recovered to safe range");
    }

    if(automationAllowed(AutomationTestSubsystem::PH) && processPHCorrection())
    {
        return;
    }

    // Automatic re-arm for a terminal EC failure latch - same architecture
    // as the PH re-arm above (see its comment for the full reasoning).
    // ecSubsystemLocked reflects either the PH_EC_CORRECTION_STALL_TIMEOUT_MS
    // budget expiring on a genuinely stalled reading, or a RESERVOIR_FULL
    // dilution block (both set it in failCurrentSubsystem()'s EC branch /
    // processECCorrection()'s dilution-blocked branch), neither of which
    // means EC is permanently unsafe. Once EC itself genuinely recovers
    // inside [minEC, maxEC], clear it here - event-driven off the real
    // reading, never a timer - together with ecAttempts, so a later,
    // independent out-of-range episode (either direction) gets its own
    // fresh budget from correctionCycleStartAt = 0. Deliberately NOT the
    // mere fact
    // that the bad side flipped - e.g. 0.80 failing then jumping straight to
    // 2.20 is still out of range on both bounds and must stay locked; only
    // an actual reading inside both minEC and maxEC counts.
    //
    // isEcCurrentlyStable() mirrors canStartNewECCorrection()'s own gate:
    // sensors.ec is the stability window's last-accepted value, retained
    // even while the live incoming signal is currently unstable and hasn't
    // reconfirmed it, so an in-range retained value alone is not sufficient
    // evidence EC is genuinely safe right now. Mock mode bypasses that
    // physical window entirely (applyEffectiveSensors() assigns
    // sensors = systemState.mockSensors directly and never feeds
    // updateStabilityWindow()), so isEcCurrentlyStable() would not reflect
    // mock's current value there - sensors.ec is already this tick's live
    // mock value, so using it directly (via this same-tick short-circuit)
    // is correct and matches how mock has always been consumed elsewhere.
    const bool ecReadingIsCurrent =
        systemState.mockSensorsEnabled || sensorManager.isEcCurrentlyStable();

    if (systemState.ecSubsystemLocked &&
        isfinite(sensors.ec) &&
        sensors.ec >= systemState.minEC &&
        sensors.ec <= systemState.maxEC &&
        ecReadingIsCurrent)
    {
        systemState.ecSubsystemLocked = false;
        systemState.ecAttempts = 0;
        systemState.ecDirection = EC_NONE;
        Serial.println("[EC] EC subsystem lock cleared - EC recovered to safe range");
        Serial.println("[EC-LOCK] cleared reason=ec_recovered_in_range");
    }

    if(automationAllowed(AutomationTestSubsystem::EC) && processECCorrection())
    {
        return;
    }

    if(fogCycleAllowed)
    {
        processFogCycle();
    }
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

    SafetyResult result =
        safetyManager.canFog();

    if(result != SafetyResult::SAFE)
    {
        actuatorManager.requestCommand(FOGGER, false, "automatic", millis());
        actuatorManager.requestCommand(BLOWER, false, "automatic", millis());

        if (!diagnosticInitialized || result != lastDiagnosticResult)
        {
            // Serial Monitor Focus Mode (see DebugManager::shouldPrintDebug()'s
            // own comment): FOGGING isolation prefers a direct, numeric block
            // reason over the generic [SAFETY] string, since fogging depends
            // on multiple sensors and "Sensor fault" alone does not say which
            // one. NONE keeps the exact original [SAFETY] line; any OTHER
            // isolated controller is unrelated and suppressed.
            if (systemState.automationTestSubsystem == AutomationTestSubsystem::NONE)
            {
                Serial.print("[SAFETY] ");
                Serial.println(safetyManager.getSafetyReason(result));
            }
            else if (debugManager.shouldPrintDebug(DebugCategory::FOGGING))
            {
                Serial.print("[FOG-BLOCK] ");
                switch (result)
                {
                    case SafetyResult::LOW_WATER:
                        Serial.print("water low: ");
                        Serial.print(sensors.waterLevelCm, 2);
                        Serial.print(" <= ");
                        Serial.println(systemState.refillStartLevelCm, 2);
                        break;
                    case SafetyResult::HIGH_WATER_TEMP:
                        Serial.print("water temperature above maximum: ");
                        Serial.print(sensors.waterTemp, 2);
                        Serial.print(" > ");
                        Serial.println(systemState.maxWaterTemp, 2);
                        break;
                    case SafetyResult::INVALID_PH:
                        if (sensors.ph < systemState.minPH)
                        {
                            Serial.print("pH below minimum: ");
                            Serial.print(sensors.ph, 2);
                            Serial.print(" < ");
                            Serial.println(systemState.minPH, 2);
                        }
                        else
                        {
                            Serial.print("pH above maximum: ");
                            Serial.print(sensors.ph, 2);
                            Serial.print(" > ");
                            Serial.println(systemState.maxPH, 2);
                        }
                        break;
                    case SafetyResult::INVALID_EC:
                        if (sensors.ec < systemState.minEC)
                        {
                            Serial.print("EC below minimum: ");
                            Serial.print(sensors.ec, 2);
                            Serial.print(" < ");
                            Serial.println(systemState.minEC, 2);
                        }
                        else
                        {
                            Serial.print("EC above maximum: ");
                            Serial.print(sensors.ec, 2);
                            Serial.print(" > ");
                            Serial.println(systemState.maxEC, 2);
                        }
                        break;
                    case SafetyResult::SENSOR_FAULT:
                        if (!isfinite(sensors.waterLevelCm)) Serial.println("water unavailable");
                        else if (!isfinite(sensors.ph)) Serial.println("pH unavailable");
                        else if (!isfinite(sensors.ec)) Serial.println("EC unavailable");
                        else Serial.println("sensor fault");
                        break;
                    default:
                        Serial.println(safetyManager.getSafetyReason(result));
                        break;
                }
            }
        }

        diagnosticInitialized = true;
        lastDiagnosticResult = result;

        return false;
    }

    if (diagnosticInitialized && lastDiagnosticResult != SafetyResult::SAFE &&
        (systemState.automationTestSubsystem == AutomationTestSubsystem::NONE ||
         debugManager.shouldPrintDebug(DebugCategory::FOGGING)))
    {
        Serial.println("[SAFETY] Normal operation restored");
    }

    diagnosticInitialized = true;
    lastDiagnosticResult = SafetyResult::SAFE;

    // REMOVED (cooling/fogging architecture update, confirmed design):
    // fogging used to also stay off for the ENTIRE duration coolingDemandActive
    // was true - i.e. for the whole band from the preventive cooling trigger
    // down to the release threshold, not just above the 28C safety ceiling.
    // That directly contradicted the confirmed design: normal cooling between
    // the preventive trigger (26.5C) and the maximum (28C) must NOT suspend
    // the programmed fog cadence - cooling and fogging are independent
    // through that whole band. The only water-temperature condition that
    // suspends fogging now is SafetyManager::canFog()'s own
    // SafetyResult::HIGH_WATER_TEMP check (waterTemp > maxWaterTemp, the
    // separate 28C safety ceiling), already evaluated above via the `result`
    // this function returned early on if unsafe - so fogging suspension
    // above 28C is still fully enforced, just through the single
    // authoritative gate instead of this second, broader, now-incorrect one.

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
    logCoolingInputSummary();

    const SafetyResult coolingSafety = safetyManager.canCool();

    // Cooling/fogging architecture update - two independent thresholds now,
    // not one derived pair:
    //   - systemState.highWaterTemp: the PREVENTIVE automatic-cooling
    //     trigger (26.5C default, HIGH_WATER_TEMP in Config.h). Despite
    //     the field's name (kept unchanged - see this task's own "trace
    //     before renaming" requirement - a full rename touches this field's
    //     Firebase key, NVS key, and every C++ call site, more risk than
    //     benefit for an un-compiled change), this is no longer "the
    //     maximum" - it is the earlier, preventive trigger.
    //   - systemState.coolerOffTemp: the cooling RELEASE threshold (25.5C
    //     default, COOLER_OFF_TEMP), independently set, no longer derived
    //     from the maximum via WATER_COOLING_HYSTERESIS. Resulting gap is
    //     now 26.5 - 25.5 = 1.0C (down from the old 2.5C), a direct,
    //     intentional consequence of moving the trigger down while keeping
    //     the release point where it was confirmed to already be.
    //   - systemState.maxWaterTemp: UNCHANGED role as the app-configurable
    //     "Water Temperature" maximum in the target-range sense, but no
    //     longer feeds cooling's own thresholds at all - it is now used
    //     exclusively as the separate upper SAFETY ceiling (28.0C default):
    //     the waterTempOutOfRange alert's own threshold, and (as of this
    //     change) SafetyManager::canFog()'s water-temperature suspension
    //     gate. 28C is "is this acceptable at all", 26.5/25.5C are "should
    //     the cooler be running right now" - genuinely different questions,
    //     no longer sharing one number.
    // Both highWaterTemp/coolerOffTemp are NO LONGER overwritten from
    // maxWaterTemp here (the previous tick-by-tick overwrite this comment
    // used to describe is removed) - they are independently Firebase/NVS-
    // authoritative again, exactly as their existing read/write/seed wiring
    // in FirebaseManager already implied but the overwrite was silently
    // defeating.
    const int8_t temperatureBand =
        sensors.waterTemp >= systemState.highWaterTemp ? 1 :
        (sensors.waterTemp <= systemState.coolerOffTemp ? -1 : 0);

    // Automatic Peltier cooling now requires an active cultivation cycle,
    // mirroring pH/EC/refill/fogging - reservoir temperature protection was
    // previously deliberately exempted from the cultivation gate ("hardware
    // protection, not cultivation"), but the confirmed design now scopes
    // AUTOMATIC cooling to an active cycle specifically, while leaving
    // sensing/alerts (AlertManager::updateWaterTemperatureAlert(), called
    // unconditionally every tick regardless of this function) and MANUAL
    // Peltier control (manualCoolingDemandActive, checked independently of
    // automaticCoolingAllowed further below) untouched. Isolated Cooling
    // Automation Test Mode is the explicit bench-testing exception -
    // automationTestSubsystem == COOLING bypasses the cultivation
    // requirement the same way it already bypasses the isolation check
    // automationAllowed() performs, so a developer can bench-test cooling
    // with no cultivation cycle created at all.
    const bool automaticCoolingAllowed =
        automationAllowed(AutomationTestSubsystem::COOLING) &&
        (systemState.automationTestSubsystem == AutomationTestSubsystem::COOLING ||
         harvestScheduleCache.isActive());

    if (!automaticCoolingAllowed || coolingSafety != SafetyResult::SAFE)
    {
        coolingDemandActive = false;
        if (coolingSafety != SafetyResult::SAFE)
        {
            manualCoolingDemandActive = false;
        }
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

    // Serial Monitor Focus Mode: [TEMP] is COOLING's own decision log.
    // [CIRCULATION] below is shared by COOLING/PH/EC (all three can demand
    // the same pump) - see DebugManager::shouldPrintDebug()'s own comment.
    // Purely print gates; every demand/state update in this function is
    // unconditional.
    const bool dbgCoolingDecision = debugManager.shouldPrintDebug(DebugCategory::COOLING);
    const bool dbgCirculation = dbgCoolingDecision ||
        debugManager.shouldPrintDebug(DebugCategory::PH) ||
        debugManager.shouldPrintDebug(DebugCategory::EC);

    if (temperatureBand != lastWaterTemperatureBand)
    {
        if (dbgCoolingDecision)
        {
            Serial.print("[TEMP] water="); Serial.print(sensors.waterTemp, 2);
            Serial.print(" high="); Serial.print(systemState.highWaterTemp, 2);
            Serial.print(" coolerOff="); Serial.println(systemState.coolerOffTemp, 2);
            if (temperatureBand == 1 && automaticCoolingAllowed &&
                coolingSafety == SafetyResult::SAFE)
                Serial.println("[TEMP] Peltier requested");
            else if (temperatureBand == -1 || coolingSafety != SafetyResult::SAFE)
                Serial.println("[TEMP] Peltier OFF requested");
        }
        lastWaterTemperatureBand = temperatureBand;
    }

    const bool phStabilizationActive =
        systemState.currentMode == STABILIZING_PH;
    const bool ecStabilizationActive =
        systemState.currentMode == STABILIZING_EC;

    // Pulse-cooling task: advance the FILL/COOL_SOAK/FLUSH state machine
    // before deciding what cooling wants from the shared circulation pump
    // this tick. manualCoolingDemandActive is entirely unaffected by this -
    // manual Peltier keeps using the pre-existing circulationConfirmed-gated
    // path further below, untouched.
    updateCoolingPulseStateMachine(automaticCoolingAllowed, coolingSafety, phStabilizationActive || ecStabilizationActive);

    // FILL and FLUSH both want circulation ON for the pulse cycle; COOL_SOAK
    // deliberately contributes nothing here so circulation can be off while
    // Peltier soaks. That exception lives narrowly in
    // ActuatorManager::validateCommand()'s PELTIER case, not in this mask -
    // this mask only ever asks for circulation ON, never forces it off out
    // from under pH/EC (see updateCoolingPulseStateMachine()'s own comment
    // on why a demand-mask OR would otherwise fight pH/EC for the pump).
    // FILL's own WAIT_CIRCULATION_OFF sub-phase is deliberately excluded:
    // that sub-phase exists specifically to let circulation actually turn
    // off before COOL_SOAK begins, so cooling must stop contributing to the
    // mask right when it starts waiting for OFF, not just once it reaches
    // COOL_SOAK - otherwise this would keep demanding circulation ON for the
    // entire time FILL is trying to confirm it's OFF, and FILL could never
    // progress (see updateCoolingPulseStateMachine()'s WAIT_CIRCULATION_OFF
    // case, which relies on exactly this to let the pump actually stop).
    const bool coolingWantsCirculation =
        (systemState.coolingPulseState == CoolingPulseState::FILL &&
         coolingPulsePhase != CoolingPulsePhase::WAIT_CIRCULATION_OFF) ||
        systemState.coolingPulseState == CoolingPulseState::FLUSH;

    uint8_t demandMask = 0;
    if (coolingWantsCirculation || manualCoolingDemandActive) demandMask |= DEMAND_PELTIER;
    if (phStabilizationActive) demandMask |= DEMAND_PH;
    if (ecStabilizationActive) demandMask |= DEMAND_EC;

    if (!circulationDiagnosticsInitialized || demandMask != lastCirculationDemandMask)
    {
        const uint8_t added = demandMask & ~lastCirculationDemandMask;
        const uint8_t removed = lastCirculationDemandMask & ~demandMask;

        if (dbgCirculation)
        {
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

    // Anchor for handleStabilizingPH()'s 10s wait: the first tick this
    // episode that circulation is actually confirmed running while PH
    // stabilization demand is active - whether that's immediate (the pump
    // was already running for cooling/EC) or delayed by actuator ramp-up.
    // Left at 0 (see the member's own comment) for every tick before that.
    if (phStabilizationActive && circulationConfirmed &&
        phStabilizationCirculationConfirmedAt == 0)
    {
        phStabilizationCirculationConfirmedAt = millis();
    }

    // Same anchor, same reasoning, for handleStabilizingEC()'s 10s wait -
    // see ecStabilizationCirculationConfirmedAt's own comment.
    if (ecStabilizationActive && circulationConfirmed &&
        ecStabilizationCirculationConfirmedAt == 0)
    {
        ecStabilizationCirculationConfirmedAt = millis();
    }

    if (circulationConfirmed != lastCirculationRunning)
    {
        if (dbgCirculation)
        {
            Serial.println(circulationConfirmed
                ? "[CIRCULATION] Pump ON"
                : "[CIRCULATION] Pump OFF");
        }
        lastCirculationRunning = circulationConfirmed;
    }

    if (circulationStatus.state != lastCirculationState)
    {
        if (circulationStatus.state == ActuatorCommandState::REJECTED && dbgCirculation)
        {
            Serial.print("[CIRCULATION] REJECTED: ");
            Serial.println(circulationStatus.reason);
        }
        lastCirculationState = circulationStatus.state;
    }

    // Manual demand keeps the exact pre-existing behavior (wait for
    // circulationConfirmed, "automatic" re-assertion - see
    // setManualCoolingDemand()'s own comment for why this is safe alongside
    // an actual manual command's own ownership; unaffected by pulse cooling).
    // Automatic cooling now goes entirely through the pulse state machine:
    // COOL_SOAK is the only state that ever requests Peltier ON, and it does
    // so unconditionally here - ActuatorManager::validateCommand()'s narrow,
    // automatic-only COOL_SOAK exception is what actually permits it to run
    // without circulation; this call site is intent, not the safety gate.
    if (manualCoolingDemandActive)
    {
        if (circulationConfirmed)
        {
            actuatorManager.requestCommand(
                PELTIER, true, "automatic", millis());
        }
        else
        {
            actuatorManager.requestCommand(
                PELTIER, false, "automatic", millis(), 100, "", "waiting_for_circulation");
        }
    }
    else if (systemState.coolingPulseState == CoolingPulseState::COOL_SOAK &&
             coolingPulsePhase == CoolingPulsePhase::NONE)
    {
        // coolingPulsePhase == NONE specifically means "actively soaking,
        // not yet asked to stop" - see updateCoolingPulseStateMachine().
        // Once that function moves to WAIT_PELTIER_OFF (soak elapsed, its
        // deadline reconciled, or pH/EC pre-emption), this condition goes
        // false on the very next tick and falls through to the OFF branch
        // below - the state machine only ever tracks state, this is the one
        // place that actually commands the actuator either way.
        actuatorManager.requestCommand(
            PELTIER, true, "automatic", millis(), 100, "cool_soak");
    }
    else
    {
        actuatorManager.requestCommand(
            PELTIER, false, "automatic", millis(), 100, "",
            coolingDemandActive ? "pulse_cooling_cycling" : "");
    }

    const bool peltierRunning = actuatorManager.getStatus(PELTIER).running;
    if (peltierRunning != lastPeltierRunning)
    {
        if (peltierRunning)
        {
            // Only genuinely true outside COOL_SOAK - during COOL_SOAK,
            // Peltier is running specifically BECAUSE circulation is
            // confirmed off (the whole point of the pulse), not because it
            // was confirmed running.
            if (systemState.coolingPulseState != CoolingPulseState::COOL_SOAK)
            {
                Serial.println("[TEMP] Circulation confirmed");
            }
            Serial.println("[TEMP] Peltier RUNNING");
        }
        else
        {
            Serial.println("[TEMP] Peltier OFF");
        }
        lastPeltierRunning = peltierRunning;
    }
}

// Pulse-cooling state machine: advances systemState.coolingPulseState and
// the internal coolingPulsePhase only - it never calls
// actuatorManager.requestCommand() itself. Every actual circulation/Peltier
// command is still issued from updateCooling()'s own tail (the
// coolingWantsCirculation demand-mask contribution and the
// coolingPulseState/coolingPulsePhase check right after it), so there is
// exactly one place in the codebase that commands either actuator for
// cooling - this function only decides what that place should do next tick.
//
// Deliberately reuses existing mechanisms instead of inventing new ones:
//   - coolingDemandActive/systemState.coolerOffTemp's existing hysteresis
//     (computed just above in updateCooling(), unchanged) decides WHEN a
//     cycle should be running at all and when FLUSH has released - not
//     re-implemented here.
//   - ActuatorStatus::forcedOffByDeadline is how a stalled loop()'s
//     independent-timer Peltier stop is detected and reconciled, exactly
//     like processPHCorrection()/handleDosingEC() already do for their own
//     pumps (see their own "Independent deadline confirmed..." handling).
//   - systemState.coolingSubsystemLocked is the existing lock RESET_SAFETY
//     already clears and SafetyManager::canCool() already checks - genuinely
//     new failure modes this state machine introduces (an invalid sensor or
//     a confirm-timeout mid-cycle) engage that same lock rather than a
//     second, parallel one.
void AutomationManager::updateCoolingPulseStateMachine(bool automaticCoolingAllowed, SafetyResult coolingSafety, bool chemistryNeedsCirculation)
{
    const bool dbgCooling = debugManager.shouldPrintDebug(DebugCategory::COOLING);

    const ActuatorStatus circulation = actuatorManager.getStatus(CIRCULATION_PUMP);
    const bool circulationRunning =
        circulation.running && circulation.state == ActuatorCommandState::RUNNING;
    const bool circulationStopped =
        !circulation.running && circulation.state == ActuatorCommandState::OFF;

    const ActuatorStatus peltier = actuatorManager.getStatus(PELTIER);
    const bool peltierStopped =
        !peltier.running && peltier.state == ActuatorCommandState::OFF;

    // Debounced the same way SafetyManager::validWaterTemperature() already
    // is (SENSOR_TRANSIENT_FAILURE_THRESHOLD consecutive invalid ticks) -
    // this check used to be a raw isfinite()/range test with no debounce,
    // which could disagree with canCool() (computed just above from the
    // debounced value) on the very same tick a transient reading occurred,
    // hard-locking cooling off a single glitch the rest of the system
    // tolerates.
    const bool waterTempRawValid =
        isfinite(sensors.waterTemp) && sensors.waterTemp >= 0.0f && sensors.waterTemp <= 100.0f;
    if (waterTempRawValid)
    {
        coolingPulseWaterTempInvalidStreak = 0;
    }
    else if (coolingPulseWaterTempInvalidStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
    {
        coolingPulseWaterTempInvalidStreak++;
    }
    const bool waterTempValid = coolingPulseWaterTempInvalidStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD;

    // Unconditional aborts - "do not silently continue the pulse cycle"
    // (task requirement). None of these let the current state finish its
    // own sequencing first.
    if (!automaticCoolingAllowed || coolingSafety != SafetyResult::SAFE)
    {
        // Mirrors updateCooling()'s own automaticCoolingAllowed/coolingSafety
        // gate above - already forces coolingDemandActive false there;
        // canCool() already covers coolingSubsystemLocked/safetyLock/invalid
        // water itself, so this is reacting to an existing signal, not a new
        // one. Just make sure a mid-cycle FILL/COOL_SOAK/FLUSH doesn't keep
        // running underneath whichever of those is now true.
        if (systemState.coolingPulseState != CoolingPulseState::IDLE)
        {
            if (dbgCooling)
            {
                Serial.print("[COOL-PULSE] Aborting to IDLE: ");
                const char* automaticBlockedReason =
                    (systemState.automationTestSubsystem != AutomationTestSubsystem::NONE &&
                     systemState.automationTestSubsystem != AutomationTestSubsystem::COOLING)
                        ? "automatic cooling isolated by test mode"
                        : "no active cultivation cycle";
                Serial.println(!automaticCoolingAllowed
                    ? automaticBlockedReason
                    : safetyManager.getSafetyReason(coolingSafety));
            }
            systemState.coolingPulseState = CoolingPulseState::IDLE;
            coolingPulsePhase = CoolingPulsePhase::NONE;
        }
        return;
    }

    if (!waterTempValid)
    {
        if (systemState.coolingPulseState != CoolingPulseState::IDLE)
        {
            if (dbgCooling)
            {
                Serial.println("[COOL-PULSE] Water-temperature reading invalid mid-cycle - locking cooling subsystem");
            }
            systemState.coolingSubsystemLocked = true;
        }
        systemState.coolingPulseState = CoolingPulseState::IDLE;
        coolingPulsePhase = CoolingPulsePhase::NONE;
        return;
    }

    // Confirm-wait stall guard, shared by every WAIT_* phase below - not a
    // tuning value, see COOLING_PULSE_CONFIRM_TIMEOUT_MS's own comment in
    // Config.h. TIMED_RUN/NONE (COOL_SOAK's own active-soak marker) are
    // deliberately excluded: those already have their own, much longer,
    // intentional duration checks below. WAIT_CIRCULATION_OFF is also
    // excluded - unlike the other two WAIT_* phases, it has no hardware
    // failure to detect here: it waits on the shared demand mask releasing
    // circulation to chemistry (pH/EC stabilization), which routinely runs
    // up to PH_STABILIZATION_TIME/EC_STABILIZATION_TIME (90s) - longer than
    // this 30s guard - during completely normal dosing, not a fault. See
    // its own case below.
    const bool waitingOnConfirmation =
        coolingPulsePhase == CoolingPulsePhase::WAIT_CIRCULATION_ON ||
        coolingPulsePhase == CoolingPulsePhase::WAIT_PELTIER_OFF;
    if (waitingOnConfirmation &&
        millis() - coolingPulsePhaseStartedAt >= COOLING_PULSE_CONFIRM_TIMEOUT_MS)
    {
        if (dbgCooling)
        {
            Serial.print("[COOL-PULSE] Stalled waiting to confirm ");
            Serial.print(coolingPulsePhase == CoolingPulsePhase::WAIT_PELTIER_OFF ? "Peltier OFF" :
                         coolingPulsePhase == CoolingPulsePhase::WAIT_CIRCULATION_ON ? "circulation ON" :
                         "circulation OFF");
            Serial.println(" - locking cooling subsystem");
        }
        systemState.coolingSubsystemLocked = true;
        systemState.coolingPulseState = CoolingPulseState::IDLE;
        coolingPulsePhase = CoolingPulsePhase::NONE;
        return;
    }

    switch (systemState.coolingPulseState)
    {
        case CoolingPulseState::IDLE:
        {
            coolingPulsePhase = CoolingPulsePhase::NONE;
            if (coolingDemandActive)
            {
                systemState.coolingPulseState = CoolingPulseState::FILL;
                coolingPulsePhase = CoolingPulsePhase::WAIT_CIRCULATION_ON;
                coolingPulsePhaseStartedAt = millis();
                if (dbgCooling) Serial.println("[COOL-PULSE] IDLE -> FILL");
            }
            break;
        }

        case CoolingPulseState::FILL:
        {
            // pH/EC wanting circulation during FILL is not a conflict - both
            // want it ON - so no pre-emption check here, only in COOL_SOAK
            // below where cooling wants it OFF.
            switch (coolingPulsePhase)
            {
                case CoolingPulsePhase::WAIT_CIRCULATION_ON:
                    if (circulationRunning)
                    {
                        coolingPulsePhase = CoolingPulsePhase::TIMED_RUN;
                        coolingPulsePhaseStartedAt = millis();
                        if (dbgCooling) Serial.println("[COOL-PULSE] FILL: circulation confirmed, timing fill");
                    }
                    break;

                case CoolingPulsePhase::TIMED_RUN:
                    if (millis() - coolingPulsePhaseStartedAt >= COOLING_PULSE_FILL_DURATION_MS_TEMP)
                    {
                        coolingPulsePhase = CoolingPulsePhase::WAIT_CIRCULATION_OFF;
                        coolingPulsePhaseStartedAt = millis();
                        if (dbgCooling) Serial.println("[COOL-PULSE] FILL complete, stopping circulation before soak");
                    }
                    break;

                case CoolingPulsePhase::WAIT_CIRCULATION_OFF:
                    // If pH/EC is (still) demanding circulation, the shared
                    // mask keeps it running regardless of what cooling wants
                    // here - this simply waits rather than fighting that
                    // demand, which is the deterministic "cooling pauses"
                    // priority the task calls for. Nothing extra to check:
                    // circulation genuinely cannot confirm OFF while
                    // chemistry holds it, so this phase just stalls until
                    // chemistry releases it - deliberately NOT bounded by
                    // the confirm-timeout above (see its exclusion comment),
                    // since a 60s pH/EC stabilization window outlasting a
                    // 30s guard is normal operation, not a stall.
                    if (circulationStopped)
                    {
                        systemState.coolingPulseState = CoolingPulseState::COOL_SOAK;
                        coolingPulsePhase = CoolingPulsePhase::NONE;
                        coolingPulsePhaseStartedAt = millis();
                        if (dbgCooling) Serial.println("[COOL-PULSE] FILL -> COOL_SOAK");
                    }
                    break;

                default:
                    break;
            }
            break;
        }

        case CoolingPulseState::COOL_SOAK:
        {
            // Highest-priority exit: pH/EC needs the pump. "End/pause the
            // cooling pulse" per the task - straight to IDLE (not FLUSH) so
            // the demand mask can hand circulation to chemistry the instant
            // Peltier confirms off; IDLE's own entry condition restarts FILL
            // later if the reservoir is still above threshold.
            if (chemistryNeedsCirculation)
            {
                if (coolingPulsePhase != CoolingPulsePhase::WAIT_PELTIER_OFF)
                {
                    coolingPulsePhase = CoolingPulsePhase::WAIT_PELTIER_OFF;
                    coolingPulsePhaseStartedAt = millis();
                    if (dbgCooling) Serial.println("[COOL-PULSE] COOL_SOAK pre-empted by pH/EC - stopping Peltier");
                }
                if (peltierStopped)
                {
                    systemState.coolingPulseState = CoolingPulseState::IDLE;
                    coolingPulsePhase = CoolingPulsePhase::NONE;
                    if (dbgCooling) Serial.println("[COOL-PULSE] COOL_SOAK -> IDLE (pH/EC priority; may resume later)");
                }
                break;
            }

            // Normal exit: soak finished, on time or via the independent
            // deadline reconciling a stalled loop() - forcedOffByDeadline is
            // the exact same reconciliation signal the pH/EC pump deadlines
            // already use (ActuatorManager's deadline-expiry block sets it;
            // AutomationManager::processPHCorrection()/handleDosingEC()
            // already read it the same way).
            if (coolingPulsePhase == CoolingPulsePhase::NONE)
            {
                const bool softTimerElapsed =
                    millis() - coolingPulsePhaseStartedAt >= COOLING_PULSE_SOAK_DURATION_MS_TEMP;
                const bool deadlineReconciled = peltier.forcedOffByDeadline;

                if (softTimerElapsed || deadlineReconciled)
                {
                    if (dbgCooling)
                    {
                        Serial.println(deadlineReconciled
                            ? "[COOL-PULSE] Independent deadline confirmed soak end (loop delayed); stopping Peltier"
                            : "[COOL-PULSE] COOL_SOAK duration elapsed, stopping Peltier");
                    }
                    coolingPulsePhase = CoolingPulsePhase::WAIT_PELTIER_OFF;
                    coolingPulsePhaseStartedAt = millis();
                }
            }
            else if (coolingPulsePhase == CoolingPulsePhase::WAIT_PELTIER_OFF && peltierStopped)
            {
                systemState.coolingPulseState = CoolingPulseState::FLUSH;
                coolingPulsePhase = CoolingPulsePhase::WAIT_CIRCULATION_ON;
                coolingPulsePhaseStartedAt = millis();
                if (dbgCooling) Serial.println("[COOL-PULSE] COOL_SOAK -> FLUSH");
            }
            break;
        }

        case CoolingPulseState::FLUSH:
        {
            switch (coolingPulsePhase)
            {
                case CoolingPulsePhase::WAIT_CIRCULATION_ON:
                    if (circulationRunning)
                    {
                        coolingPulsePhase = CoolingPulsePhase::TIMED_RUN;
                        coolingPulsePhaseStartedAt = millis();
                        if (dbgCooling) Serial.println("[COOL-PULSE] FLUSH: circulation confirmed, timing flush");
                    }
                    break;

                case CoolingPulsePhase::TIMED_RUN:
                    if (millis() - coolingPulsePhaseStartedAt >= COOLING_PULSE_FLUSH_DURATION_MS_TEMP)
                    {
                        // Reuse the existing hysteresis flag (coolingDemandActive,
                        // computed once above from sensors.waterTemp vs.
                        // systemState.coolerOffTemp) rather than re-comparing
                        // the reading ourselves - one definition of the
                        // release threshold.
                        if (!coolingDemandActive)
                        {
                            systemState.coolingPulseState = CoolingPulseState::IDLE;
                            coolingPulsePhase = CoolingPulsePhase::NONE;
                            if (dbgCooling) Serial.println("[COOL-PULSE] FLUSH complete, released <= threshold -> IDLE");
                        }
                        else
                        {
                            systemState.coolingPulseState = CoolingPulseState::FILL;
                            coolingPulsePhase = CoolingPulsePhase::WAIT_CIRCULATION_ON;
                            if (dbgCooling) Serial.println("[COOL-PULSE] FLUSH complete, still above threshold -> FILL (repeat)");
                        }
                        coolingPulsePhaseStartedAt = millis();
                    }
                    break;

                default:
                    break;
            }
            break;
        }
    }
}

bool AutomationManager::isCirculationRequired() const
{
    return lastCirculationDemandMask != 0;
}

// User-facing explanation for a refused manual OFF. Reports the most
// safety-relevant demand first and never exposes the mask itself.
const char* AutomationManager::circulationRequirementReason() const
{
    if (lastCirculationDemandMask & DEMAND_PELTIER)
        return "Circulation is required during water cooling.";
    if (lastCirculationDemandMask & DEMAND_PH)
        return "Circulation is required during pH stabilization.";
    if (lastCirculationDemandMask & DEMAND_EC)
        return "Circulation is required during EC stabilization.";
    return "Circulation is required by an active automatic operation.";
}

String AutomationManager::getCirculationReason(uint8_t demandMask) const
{
    String reason;
    if (demandMask & DEMAND_PELTIER) reason = "temperature_circulation";
    if (demandMask & DEMAND_PH)
    {
        if (!reason.isEmpty()) reason += "+";
        reason += "ph_stabilization";
    }
    if (demandMask & DEMAND_EC)
    {
        if (!reason.isEmpty()) reason += "+";
        reason += "ec_stabilization";
    }
    return reason;
}

//PH Correction Handling
bool AutomationManager::processPHCorrection()
{
    // Serial Monitor Focus Mode: printed BEFORE the decision below (see
    // logPHInputSummary()'s own comment) so isolated PH testing sees the
    // inputs the decision is about to be made from, not just its outcome.
    // Edge-triggered on input change; this function is already only
    // called under PH isolation or NONE (see automationAllowed()), so no
    // further mode gating is needed here.
    logPHInputSummary();

    if(systemState.phSubsystemLocked)
        return false;

    if(!alertState.phOutOfRange)
    {
        logPHDecisionLine(
            "[PH] value=" + String(sensors.ph, 2) +
            " range=" + String(systemState.minPH, 2) + "-" + String(systemState.maxPH, 2) +
            " -> NO CORRECTION");
        return false;
    }

    if(!canStartNewPHCorrection())
    {
        logPHDecisionLine("[PH-BLOCK] pH unstable");
        return false;
    }

    // sensors.ph is the stable-value filter's authoritative output (see
    // SensorManager::applyEffectiveSensors()) - the same value phOutOfRange
    // above was derived from, so this plain comparison against minPH can't
    // disagree with the trigger it's gated behind.
    systemState.phDirection =
        sensors.ph < systemState.minPH ? PH_UP : PH_DOWN;

    systemState.phDoseTime =
        PH_DOSING_TIME;

    SafetyResult result =
        safetyManager.canDosePH();

    if(result != SafetyResult::SAFE)
    {
        // A safety-blocked attempt did not take over this tick - returning
        // true here (as this used to) told handleNormal() "I'm handling
        // this, stop" identically to an actual dose starting. With
        // phOutOfRange persistently true (e.g. LOW_WATER blocking a dose the
        // whole time water stays low), that starved everything after this
        // check forever: processECCorrection() in the same caller never even
        // got evaluated, and processFogCycle() never fell through to its own
        // logic. failCurrentOperation() itself is a no-op here
        // (operationRequest.state is never RUNNING at this call site - see its
        // own guard), kept only as a defensive marker if that ever changes.
        logPHDecisionLine(
            result == SafetyResult::LOW_WATER ? "[PH-BLOCK] water low" :
            result == SafetyResult::SENSOR_FAULT && !isfinite(sensors.waterLevelCm) ? "[PH-BLOCK] water unavailable" :
            "[PH-BLOCK] " + String(safetyManager.getSafetyReason(result)));

        failCurrentOperation(
            safetyManager.getSafetyReason(result));

        return false;
    }

    createOperationRequest(
        generateAutoRequestId(),
        systemState.phDirection == PH_UP ? OperationType::PH_UP : OperationType::PH_DOWN,
        OperationAction::START,
        RequestSource::AUTOMATIC
    );

    // Compact single-line decision (Serial Monitor Focus Mode) - replaces
    // the previous two-line [PH] value=.../[PH] requesting... pair with the
    // same information. This is a one-shot print: reaching here means a
    // correction is actually starting, and changeState(DOSING_PH) below
    // leaves NORMAL, so processPHCorrection() is not called again until
    // this correction completes.
    Serial.print("[PH] value=");
    Serial.print(sensors.ph, 2);
    if (systemState.phDirection == PH_UP)
    {
        Serial.print(" min=");
        Serial.print(systemState.minPH, 2);
    }
    else
    {
        Serial.print(" max=");
        Serial.print(systemState.maxPH, 2);
    }
    Serial.print(" -> ");
    Serial.println(systemState.phDirection == PH_UP ? "PH_UP" : "PH_DOWN");

    systemState.correctionMode =
    CorrectionMode::AUTOMATIC;

    systemState.firstCorrectionCycle = true;

    systemState.phAttempts = 0;

    // Quiet-monitoring/4-minute-budget redesign: only initialize a fresh
    // episode when one is not already running. correctionCycleStartAt is
    // shared with EC (see its own comment, Types.h) but currentMode can
    // only ever be in a pH or an EC state at once, never both, so this is
    // safe. A guard on == 0 (not on firstCorrectionCycle above, which an
    // internal redose also leaves true-then-false across its own tighter
    // scope) ensures an internal redose - handleStabilizingPH() calling
    // changeState(DOSING_PH) directly, never back through this function -
    // never resets the budget clock or the checkpoint bookkeeping.
    if (systemState.correctionCycleStartAt == 0)
    {
        systemState.correctionCycleStartAt = millis();
        systemState.phTrendReferenceValue = NAN;
        systemState.phLastTrendCheckAt = 0;
        systemState.phLastTrendImproving = true;
        systemState.phFirstCheckpointPublished = false;
        systemState.phStableSince = 0;
        systemState.phStableCheckpointPublished = false;
    }

    changeState(
        DOSING_PH);

    return true;
}

//EC Correction Handling
bool AutomationManager::processECCorrection()
{
    // Serial Monitor Focus Mode: see processPHCorrection()'s matching
    // comment - printed before the decision below, edge-triggered on input
    // change, already scoped to EC isolation/NONE by this function's own
    // caller (automationAllowed(EC)).
    logECInputSummary();

    if(systemState.ecSubsystemLocked)
        return false;

    if(!alertState.ecLow && !alertState.ecHigh)
    {
        logECDecisionLine(
            "[EC] value=" + String(sensors.ec, 2) +
            " range=" + String(systemState.minEC, 2) + "-" + String(systemState.maxEC, 2) +
            " -> NO CORRECTION");
        return false;
    }

    if(!canStartNewECCorrection())
    {
        logECDecisionLine("[EC-BLOCK] EC unstable");
        return false;
    }

    systemState.ecDirection = alertState.ecLow ? EC_RAISE : EC_DILUTE;

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
            Serial.println("[EC-LOCK] set reason=reservoir_full_dilution_blocked");
        }
        else
        {
            logECDecisionLine(
                result == SafetyResult::LOW_WATER ? "[EC-BLOCK] water low" :
                result == SafetyResult::SENSOR_FAULT && !isfinite(sensors.waterLevelCm) ? "[EC-BLOCK] water unavailable" :
                "[EC-BLOCK] " + String(safetyManager.getSafetyReason(result)));
        }

        failCurrentOperation(
            safetyManager.getSafetyReason(result));

        // See processPHCorrection()'s matching comment: a safety-blocked
        // attempt must report false, not true, or it starves handleNormal()'s
        // processFogCycle() fallthrough for as long as ecLow/ecHigh stays true.
        return false;
    }

    createOperationRequest(
        generateAutoRequestId(),
        OperationType::EC_CORRECTION,
        OperationAction::START,
        RequestSource::AUTOMATIC
    );

    // Compact single-line decision (Serial Monitor Focus Mode) - see
    // processPHCorrection()'s matching comment; a one-shot print, since
    // changeState(DOSING_EC) below leaves NORMAL.
    Serial.print("[EC] value=");
    Serial.print(sensors.ec, 2);
    if (systemState.ecDirection == EC_RAISE)
    {
        Serial.print(" min=");
        Serial.print(systemState.minEC, 2);
    }
    else
    {
        Serial.print(" max=");
        Serial.print(systemState.maxEC, 2);
    }
    Serial.print(" -> ");
    Serial.println(systemState.ecDirection == EC_RAISE ? "NUTRIENT DOSE" : "DILUTION");

    systemState.correctionMode =
    CorrectionMode::AUTOMATIC;

    systemState.firstCorrectionCycle = true;
    systemState.ecAttempts = 0;

    // Set only now that DOSING_EC is actually about to start - not earlier,
    // where a safety rejection above would have left it stuck at
    // EC_DOSING_TIME while currentMode stayed NORMAL and nothing was dosing.
    systemState.ecDoseTime = EC_DOSING_TIME;

    // Quiet-monitoring/4-minute-budget redesign - see processPHCorrection()'s
    // matching comment for the full reasoning (correctionCycleStartAt is
    // shared between pH and EC; the == 0 guard is what makes an internal
    // redose never reset it).
    if (systemState.correctionCycleStartAt == 0)
    {
        systemState.correctionCycleStartAt = millis();
        systemState.ecTrendReferenceValue = NAN;
        systemState.ecLastTrendCheckAt = 0;
        systemState.ecLastTrendImproving = true;
        systemState.ecFirstCheckpointPublished = false;
        systemState.ecStableSince = 0;
        systemState.ecStableCheckpointPublished = false;
    }

    changeState(
        DOSING_EC);

    return true;
}

// Gate for starting a NEW pH/EC correction - see AutomationManager.h's
// comment on these and StabilityWindow::currentlyStable (SensorManager.h)
// for the full design. Deliberately just this one check: "a valid
// lastStablePH/EC exists" and "existing normal safety conditions pass" are
// already enforced at every call site independently (alertState.phOutOfRange
// /ecLow/ecHigh already require a finite sensors.ph/ec, and canDosePH()/
// canDoseEC()/canDiluteEC() already reject a NaN reading as SENSOR_FAULT) -
// this adds only the missing condition: the CURRENT stability window must
// have just reconfirmed the reading, not merely be retaining an old one.
bool AutomationManager::canStartNewPHCorrection() const
{
    // Mock values are supplied directly by the developer and bypass the
    // physical stability window in SensorManager::applyEffectiveSensors().
    // Requiring that physical window here made mock pH tests depend on stale
    // hardware state. Validity and all dosing safety checks still run at the
    // call sites before an operation starts and throughout dosing. Mock also
    // bypasses PH_DOSE_COOLDOWN below for the same reason - there is no real
    // chemical mixing delay to wait out on a developer-supplied value.
    if (systemState.mockSensorsEnabled) return true;

    // A settled reading is not proof the dosed chemical has actually finished
    // mixing into the reservoir - the probe can report a steady value before
    // that's true. PH_DOSE_COOLDOWN is a real minimum wait since the last
    // pH-Up/pH-Down pump run (either source - see the pump-off funnel in
    // ActuatorManager::update() that sets lastPhDoseEndedAt), enforced on top
    // of, not instead of, the stability window below. 0 is the "never dosed
    // this boot" sentinel, not a real timestamp.
    if (systemState.lastPhDoseEndedAt != 0 &&
        millis() - systemState.lastPhDoseEndedAt < PH_DOSE_COOLDOWN)
    {
        return false;
    }

    return sensorManager.isPhCurrentlyStable();
}

bool AutomationManager::canStartNewECCorrection() const
{
    // Same controlled-source rule as pH above. A finite, validated mock EC
    // payload is current by definition; physical EC retains the full window.
    if (systemState.mockSensorsEnabled) return true;

    // Same reasoning as canStartNewPHCorrection() above, for the Grow/Bloom
    // pumps and EC_DOSE_COOLDOWN.
    if (systemState.lastEcDoseEndedAt != 0 &&
        millis() - systemState.lastEcDoseEndedAt < EC_DOSE_COOLDOWN)
    {
        return false;
    }

    return sensorManager.isEcCurrentlyStable();
}

// See the header's own comment. Deliberately re-derives its answer from the
// same fields processPHCorrection()/processECCorrection() already check
// (alertState.xLow/xHigh, canStartNewXCorrection(), canDoseX()/canDiluteX())
// rather than threading a reason code back out of those functions - this
// stays purely observational and cannot change what they decide. A
// SafetyResult::SENSOR_FAULT is expanded to name the SPECIFIC invalid
// sensor (per this task's own instruction to avoid a bare "SENSOR_FAULT"
// that doesn't say which reading is the problem) using the same sensors.*
// finiteness checks SafetyManager's validPH()/validEC()/validWaterLevel()
// are built on.
void AutomationManager::logAutomationTestBlockReason()
{
    static unsigned long lastLogAt = 0;
    const unsigned long now = millis();
    if (lastLogAt != 0 && now - lastLogAt < AUTO_TEST_BLOCK_LOG_INTERVAL_MS)
    {
        return;
    }

    const AutomationTestSubsystem selected = systemState.automationTestSubsystem;

    if (selected == AutomationTestSubsystem::PH)
    {
        const char* reason = nullptr;

        if (systemState.phSubsystemLocked)
        {
            reason = "PH_SUBSYSTEM_LOCKED";
        }
        else if (!alertState.phOutOfRange)
        {
            // Inside range - nothing to correct, not a block.
        }
        else if (!canStartNewPHCorrection())
        {
            reason = "PH_NOT_STABLE";
        }
        else
        {
            const SafetyResult result = safetyManager.canDosePH();
            if (result == SafetyResult::SENSOR_FAULT)
            {
                reason = !isfinite(sensors.waterLevel) ? "WATER_LEVEL_INVALID" : "PH_INVALID";
            }
            else if (result == SafetyResult::LOW_WATER) reason = "LOW_WATER";
            else if (result == SafetyResult::RESERVOIR_LOCK) reason = "RESERVOIR_LOCKED_BY_ANOTHER_OPERATION";
            else if (result == SafetyResult::SAFETY_LOCKED) reason = "HARD_SAFETY_LOCK";
            else if (result != SafetyResult::SAFE) reason = "PH_SAFETY_BLOCK";
        }

        if (reason != nullptr)
        {
            lastLogAt = now;
            Serial.print("[AUTO-TEST-BLOCK] subsystem=PH reason=");
            Serial.println(reason);
        }
    }
    else if (selected == AutomationTestSubsystem::EC)
    {
        const char* reason = nullptr;

        if (systemState.ecSubsystemLocked)
        {
            reason = "EC_SUBSYSTEM_LOCKED";
        }
        else if (!alertState.ecLow && !alertState.ecHigh)
        {
            // Inside range - nothing to correct, not a block.
        }
        else if (!canStartNewECCorrection())
        {
            reason = "EC_NOT_STABLE";
        }
        else
        {
            const SafetyResult result = alertState.ecLow
                ? safetyManager.canDoseEC()
                : safetyManager.canDiluteEC();
            if (result == SafetyResult::SENSOR_FAULT)
            {
                reason = !isfinite(sensors.waterLevel) ? "WATER_LEVEL_INVALID" : "EC_INVALID";
            }
            else if (result == SafetyResult::LOW_WATER) reason = "LOW_WATER";
            else if (result == SafetyResult::RESERVOIR_LOCK) reason = "RESERVOIR_LOCKED_BY_ANOTHER_OPERATION";
            else if (result == SafetyResult::RESERVOIR_FULL) reason = "RESERVOIR_FULL";
            else if (result == SafetyResult::SUBSYSTEM_LOCKED) reason = "REFILL_SUBSYSTEM_LOCKED";
            else if (result == SafetyResult::SAFETY_LOCKED) reason = "HARD_SAFETY_LOCK";
            else if (result != SafetyResult::SAFE) reason = "EC_SAFETY_BLOCK";
        }

        if (reason != nullptr)
        {
            lastLogAt = now;
            Serial.print("[AUTO-TEST-BLOCK] subsystem=EC reason=");
            Serial.println(reason);
        }
    }
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
            Serial.println("[EC-LOCK] set reason=reservoir_full_dilution_blocked");
        }
        failCurrentOperation(
            safetyManager.getSafetyReason(result));

        return;
    }

    if(!canStartNewECCorrection())
    {
        failCurrentOperation(
            "EC reading is not currently stable; retry once a fresh stable reading is confirmed.");

        return;
    }

    systemState.correctionMode =
        systemState.operationRequest.source == RequestSource::AUTOMATIC ?
            CorrectionMode::AUTOMATIC : CorrectionMode::MANUAL;

        systemState.firstCorrectionCycle = true;
        systemState.ecAttempts = 0;

    // Set only now that DOSING_EC is actually about to start - see
    // processECCorrection()'s matching comment.
    systemState.ecDoseTime = EC_DOSING_TIME;

    changeState(
        DOSING_EC);
}

//Fog Cycle Handling
void AutomationManager::processFogCycle()
{
    unsigned long elapsed =
        millis() -
        fogTimerStart;

    // NORMAL fallback cadence whenever DHT is unavailable/stale (see the
    // automation resilience pass report) - DHT selects cadence only, it
    // never gates fogging permission (SafetyManager::canFog()). Checked via
    // dhtAvailable, not isnan(sensors.temperature): a stale reading is
    // finite (held last-good) but must not be trusted to pick hot/cold
    // cadence, since it may no longer reflect current conditions.
    String fogStrategy = "normal";

    if(sensors.dhtAvailable)
    {
        // Inclusive boundaries: >= hotFogTemperature is HOT, <=
        // coldFogTemperature is COLD, otherwise NORMAL. See
        // HOT_FOG_TEMPERATURE/COLD_FOG_TEMPERATURE in Config.h.
        if(sensors.temperature >=
           systemState.hotFogTemperature)
        {
            fogStrategy =
                "hot";
        }
        else if(sensors.temperature <=
                systemState.coldFogTemperature)
        {
            fogStrategy =
                "cold";
        }
    }

    if(activeFogStrategy == "")
    {
        if (!sensors.dhtAvailable)
        {
            Serial.println("[FOG] DHT unavailable -> NORMAL cadence fallback");
        }

        activeFogStrategy =
            fogStrategy;
    }

    // Serial Monitor Focus Mode compact dependency summary (see
    // AutomationManager::logFogInputSummary()'s own comment) - cadence label
    // matches the [FOG] fallback line above: NORMAL_FALLBACK only when the
    // active "normal" strategy is standing in for DHT being unavailable, not
    // for a genuinely DHT-selected normal cadence.
    {
        const char* cadenceLabel =
            activeFogStrategy == "hot" ? "HOT" :
            activeFogStrategy == "cold" ? "COLD" :
            !sensors.dhtAvailable ? "NORMAL_FALLBACK" : "NORMAL";
        logFogInputSummary(cadenceLabel);
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

        // CONFIRMED BUG FIX (root-blower/canopy-fan speed separation): the
        // root-zone Blower used to borrow lastAutomaticCanopySpeed - the
        // CANOPY_FAN's own temp/humidity-derived speed (see
        // handleCanopyClimate()'s 70/100/50% hysteresis) - meaning root-zone
        // airflow through the PVC/root chamber changed whenever canopy
        // conditions changed, with no independent identity of its own. The
        // root blower moving fog to the roots and the canopy fan moving air
        // around the grow space are physically and logically separate
        // actuators serving different purposes; the confirmed design gives
        // the root blower its own fixed automatic fogging speed
        // (systemState.blowerSpeedPercent, default BLOWER_SPEED_DEFAULT_PERCENT
        // = 65% - see Config.h's own comment for the full reasoning),
        // independent of canopy temperature demand, humidity demand, and
        // CANOPY_FAN's PWM. Only this ON case (the fogger/blower pair
        // actively running) uses it; the OFF/purge branch below is a
        // separate, deliberately-untouched mechanism at its own fixed 100%,
        // not "the pair running".
        actuatorManager.requestCommand(
            BLOWER, true, "automatic", millis(), systemState.blowerSpeedPercent, activeFogStrategy);

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

        // Blower purge: stays on for the first BLOWER_PURGE_MS of the
        // OFF/rest window to clear fog concentrated near the reservoir
        // toward the root chamber, then off for the remainder of the
        // unchanged fogOffTime window - elapsed is already measured from
        // the same ON->OFF transition instant, so this never extends the
        // total NORMAL/HOT/COLD cycle length. The fogger is already off for
        // the whole purge window by design, so the blower's automatic
        // fogger-running gate is waived here (see
        // ActuatorManager::validateCommand's BLOWER case).
        //
        // Deliberately fixed at 100%, NOT systemState.blowerSpeedPercent:
        // this is a purge/clearing phase, not the configured normal
        // fogging airflow, so it intentionally always runs at full speed
        // regardless of the configured automatic fogging percentage.
        {
            // Preserve the production purge in Off / Full System. Isolated
            // Fogging tests keep the automatic pair exact: Fogger OFF means
            // Blower OFF for the whole configured rest interval.
            bool purging =
                systemState.automationTestSubsystem != AutomationTestSubsystem::FOGGING &&
                elapsed < BLOWER_PURGE_MS;
            actuatorManager.requestCommand(
                BLOWER, purging, "automatic", millis(), 100, activeFogStrategy, "", false, purging);
        }

        if(elapsed >= fogOffTime)
        {
            fogCycleOn = true;
            fogTimerStart = millis();
            activeFogStrategy = "";
        }
    }
}

// Manual root-fogging redesign: a normal Admin manual Fogger request
// represents the complete root-fogging function (Fogger + root-zone Blower
// together), not raw Fogger-only hardware testing. This does NOT touch
// fogCycleOn/fogTimerStart/activeFogStrategy - the automatic fog-cycle timer
// keeps running (or not) exactly as it already does under any other manual
// override, so a manual root-fogging request can never start or corrupt the
// automatic cycle's own state; ActuatorManager::requestCommand()'s existing
// manual-outranks-automatic guard is what prevents the automatic FOGGER/
// BLOWER commands from taking effect while manually held.
//
// Reads FOGGER's own already-validated status this same tick (FOGGER
// precedes BLOWER in ActuatorManager's array-index order, so by the time
// this runs next tick its status already reflects that tick's
// validateCommand() outcome - see ActuatorManager.cpp's FOGGER case, now
// gated by canFog() for manual too) rather than blindly mirroring the
// request, so a rejected or not-yet-running manual Fogger command never
// drags the Blower on with it.
void AutomationManager::processManualFogPairing()
{
    const ActuatorStatus foggerStatus = actuatorManager.getStatus(FOGGER);
    const bool foggerManuallyRunning =
        foggerStatus.source == "manual" &&
        foggerStatus.running &&
        foggerStatus.state == ActuatorCommandState::RUNNING;

    if (foggerManuallyRunning)
    {
        manualFogPurgeActive = false;
        wasManualFoggerRunning = true;

        // Base manual root-fogging blower speed is the same fixed
        // automatic fogging speed (systemState.blowerSpeedPercent, default
        // 65% - see Config.h). Existing fan-assisted environmental demand
        // still takes priority for maximum airflow (100%):
        //
        // - Temperature: CONFIRMED FIX - compared directly against
        //   systemState.hotFogTemperature (the same authoritative Hot
        //   fogging threshold processFogCycle() uses to pick "hot" cadence,
        //   >=32C by default), using this tick's live sensors.temperature.
        //   Deliberately NOT highAirDemandActive: that is CANOPY_FAN's own,
        //   differently-thresholded (>highAirTemp trigger / <=airTempRelease
        //   release) hysteresis latch, which can sit well below (or, once
        //   released, well after) the Hot fogging threshold and would give
        //   the wrong 100%/65% decision here. This check has no hysteresis
        //   of its own by design - a plain >=/< comparison re-evaluated
        //   fresh every tick, exactly matching how processFogCycle() itself
        //   selects hot/normal/cold cadence. Gated on sensors.dhtAvailable
        //   so a stale/unavailable reading is never treated as a fresh
        //   >=hotFogTemperature reading - falls through to the humidity
        //   check (and otherwise the 65% baseline) instead, the same
        //   "DHT selects/limits demand, never fabricates it" principle used
        //   throughout this codebase (see processFogCycle()'s own comment).
        // - Humidity: unchanged, still highHumidityDemandActive (trigger
        //   >75% RH / release <=70% RH, preserved as-is, including that it
        //   stays latched at its last value while DHT is unavailable).
        const bool freshHotAirTemp =
            sensors.dhtAvailable && sensors.temperature >= systemState.hotFogTemperature;
        const uint8_t rootBlowerSpeed =
            (freshHotAirTemp || highHumidityDemandActive) ? 100 : systemState.blowerSpeedPercent;

        actuatorManager.requestCommand(
            BLOWER, true, "manual", millis(), rootBlowerSpeed);
        return;
    }

    if (wasManualFoggerRunning)
    {
        // Manual Fogger just stopped (Admin request, its own manual deadline,
        // or Manual Mode expiry) - begin the same BLOWER_PURGE_MS clearing
        // purge the automatic fog cycle already performs, at a fixed 100%
        // (see processFogCycle()'s matching purge comment), before turning
        // the root blower off.
        wasManualFoggerRunning = false;
        manualFogPurgeActive = true;
        manualFogPurgeStart = millis();
    }

    if (manualFogPurgeActive)
    {
        if (millis() - manualFogPurgeStart < BLOWER_PURGE_MS)
        {
            actuatorManager.requestCommand(BLOWER, true, "manual", millis(), 100);
        }
        else
        {
            manualFogPurgeActive = false;
            actuatorManager.requestCommand(BLOWER, false, "manual", millis());
        }
    }
}

//Grow Light Schedule Handling
// Strict activation condition for the developer-only Grow Light mock time -
// see the header's own comment. Both conditions gate independently: leaving
// GROW_LIGHT test mode (even with the Firebase flag still stored true, by
// design - see the app-side task note that a stale value need not be
// deleted) or disabling the flag while still in GROW_LIGHT test mode both
// immediately fall back to real RTC behavior on the very next tick, since
// this is re-evaluated fresh every call rather than latched.
bool AutomationManager::growLightMockTimeActive() const
{
    return systemState.automationTestSubsystem == AutomationTestSubsystem::GROW_LIGHT &&
        systemState.mockGrowLightTimeEnabled;
}

void AutomationManager::updateGrowLightSchedule()
{
    // CONFIRMED BUG FIX (grow-light edge-case correction): getHour()/
    // getMinute() only check RTCManager::isConnected(), not hasValidTime() -
    // so a DS3231 that lost power still returns whatever it currently
    // reads, and this schedule would silently run against that garbage
    // "current time" instead of the real one. This USED TO simply return
    // here, leaving the grow light in whatever state it last happened to be
    // commanded - which could mean an automatically-running light stays ON
    // indefinitely while the clock is untrustworthy, with no way to know if
    // that is still correct. Now fails safe: automatic control is
    // explicitly commanded OFF instead (see below) rather than merely
    // frozen. Still "automatic" source, so an authorized manual hold still
    // outranks this exactly per the existing hard-safety/manual/automatic
    // priority architecture - this only changes what the AUTOMATIC signal
    // itself asks for, never adds a new hard prohibition.
    //
    // The one narrow exception: the developer-only Grow Light mock time
    // (growLightMockTimeActive()) supplies a deterministic test "current
    // time" of its own via getCurrentMinutes() below, so an invalid/
    // unavailable physical RTC must not block scheduling in that case -
    // this never widens what counts as a valid REAL RTC reading; it only
    // adds a second, narrowly-gated way to proceed. See the header's own
    // comment for the exact activation condition, and REQUIRED EFFECTIVE
    // BEHAVIOR in this task for why Full System and every other test-mode
    // selection must never observe this bypass.
    if (!rtcManager.hasValidTime() && !growLightMockTimeActive())
    {
        actuatorManager.requestCommand(
            GROW_LIGHT, false, "automatic", millis(), 100, "",
            "Automatic lighting paused: RTC time is not currently valid");
        return;
    }

    bool lightEnabled =
        isWithinSchedule(
            systemState.lightOnHour,
            systemState.lightOnMinute,
            systemState.lightOffHour,
            systemState.lightOffMinute);

    // Diagnostic only - isWithinSchedule() above is the single authoritative
    // place the equal-ON/OFF-time rejection actually happens (already
    // returned false for this case). This is purely so the OFF command
    // below can carry a specific, human-readable reason distinguishing
    // "invalid schedule" from an ordinary "currently outside the window" -
    // it does not re-decide anything isWithinSchedule() already decided.
    const bool scheduleInvalid =
        systemState.lightOnHour == systemState.lightOffHour &&
        systemState.lightOnMinute == systemState.lightOffMinute;

    // Serial Monitor Focus Mode: one compact line per decision change, not
    // every tick - see DebugManager::shouldPrintDebug()'s own comment. No
    // dedicated diagnostic previously existed here; purely additive.
    if (debugManager.shouldPrintDebug(DebugCategory::LIGHT))
    {
        static bool lightLogInitialized = false;
        static bool lastLoggedLightEnabled = false;
        static bool lastLoggedMockActive = false;
        static uint16_t lastLoggedMinutes = 0xFFFF;

        const bool mockActive = growLightMockTimeActive();
        const int currentMinutes = getCurrentMinutes();

        if (!lightLogInitialized || lightEnabled != lastLoggedLightEnabled ||
            mockActive != lastLoggedMockActive ||
            (uint16_t)currentMinutes != lastLoggedMinutes)
        {
            lightLogInitialized = true;
            lastLoggedLightEnabled = lightEnabled;
            lastLoggedMockActive = mockActive;
            lastLoggedMinutes = (uint16_t)currentMinutes;

            Serial.print("[RTC] time=");
            Serial.print(rtcManager.getHour());
            Serial.print(":");
            Serial.print(rtcManager.getMinute());
            Serial.print(" valid=");
            Serial.print(rtcManager.hasValidTime() ? "true" : "false");
            Serial.print(" mockTime=");
            Serial.println(mockActive ? "true" : "false");

            Serial.print("[LIGHT] schedule=");
            Serial.print(systemState.lightOnHour);
            Serial.print(":");
            Serial.print(systemState.lightOnMinute);
            Serial.print("-");
            Serial.print(systemState.lightOffHour);
            Serial.print(":");
            Serial.print(systemState.lightOffMinute);
            Serial.print(" now=");
            Serial.print(currentMinutes / 60);
            Serial.print(":");
            Serial.print(currentMinutes % 60);
            Serial.print(" -> ");
            Serial.println(lightEnabled ? "ON" : "OFF");
        }
    }

    if(lightEnabled)
    {
        actuatorManager.requestCommand(
            GROW_LIGHT, true, "automatic", millis());
    }
    else
    {
        // Existing actuator-status "reason" field (already published to
        // Firebase alongside every other actuator's on/off state, e.g. the
        // "waiting_for_circulation"/deadline-reconciliation reasons used
        // elsewhere in this file) is reused here rather than adding a new
        // dedicated settings/status field for this diagnostic - an ordinary
        // "currently outside the window" OFF carries no reason, same as
        // before, only the specifically-invalid-schedule case does.
        actuatorManager.requestCommand(
            GROW_LIGHT, false, "automatic", millis(), 100, "",
            scheduleInvalid
                ? "Automatic lighting schedule invalid: ON time equals OFF time"
                : "");
    }
}
//Get Current Time in Minutes
int AutomationManager::getCurrentMinutes() const
{
    // See growLightMockTimeActive()'s own comment for the strict activation
    // condition. Everywhere else (Full System, every other Automation Test
    // Mode selection, or the flag disabled) falls through unchanged to the
    // real RTC read below - the DS3231 itself is never touched by this
    // branch either way.
    if (growLightMockTimeActive())
    {
        return systemState.mockGrowLightMinutes;
    }

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

    // CONFIRMED BUG FIX (grow-light edge-case correction): ON time == OFF
    // time fell through to the "overnight schedule" branch below (start <
    // end is false when they're equal), where current >= start || current <
    // end is true for EVERY possible current value - i.e. an equal
    // ON/OFF schedule silently meant "always on, 24 hours a day," never
    // intended. The single, central point every caller already goes
    // through (this function has exactly one caller,
    // updateGrowLightSchedule()) - an equal ON/OFF pair is not a schedule
    // at all and must never be interpreted as either a valid normal or
    // valid overnight one.
    if(start == end)
    {
        return false;
    }

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
void AutomationManager::resetAutomaticRefillAttempts()
{
    automaticRefillPhase = AutomaticRefillPhase::RUNNING;
    automaticRefillAttempt = 1;
    automaticRefillPhaseStartedAt = 0;
    automaticRefillAttemptStartLevel = sensors.waterLevelCm;
}

void AutomationManager::completeRefillSuccess()
{
    const bool dbgWater = debugManager.shouldPrintDebug(DebugCategory::WATER);
    if (dbgWater) Serial.println("[REFILL] Stop threshold reached");
    actuatorManager.requestCommand(SOLENOID, false, "automatic", millis(), 100, "refill");
    if (dbgWater) Serial.println("[REFILL] Solenoid OFF");

    systemState.reservoirLocked = false;
    completeCurrentOperation();
    if (dbgWater) Serial.println("[REFILL] Operation COMPLETED");

    if(systemState.automationTestSubsystem == AutomationTestSubsystem::REFILL)
    {
        // Isolated REFILL test: a successful refill must not hand off to
        // STARTUP's fog sequence - that would leak outside REFILL-only
        // isolation into Fogger/Blower and every other automatic actuator.
        // Land in NORMAL instead, where handleNormal()'s automationAllowed()
        // gates keep every unrelated controller paused for as long as REFILL
        // test mode stays selected. Full System (test mode NONE) is
        // unaffected - it still proceeds to STARTUP below.
        changeState(NORMAL);
        return;
    }

    changeState(STARTUP);
}

bool AutomationManager::handleBoundedAutomaticRefill()
{
    // Bounded 3-attempt policy is the production automatic-refill lifecycle
    // (promoted from what was originally a REFILL-isolation-test-only
    // contract): run the solenoid for AUTOMATIC_REFILL_RUN_TIME, then let
    // the reading settle for AUTOMATIC_REFILL_SETTLE_TIME before trusting
    // it - the ultrasonic sensor reads unreliably while water is actively
    // flowing into the reservoir - and give up after MAX_REFILL_ATTEMPTS
    // rather than holding the solenoid open continuously for up to
    // OPERATION_TIMEOUT_MS. Manual refill requests (an admin's "Start
    // Reservoir Refill" button) are deliberately excluded - an operator
    // watching the refill happen can stop it manually if something looks
    // wrong, so manual keeps the simpler continuous-run behavior below.
    if(systemState.operationRequest.source != RequestSource::AUTOMATIC)
    {
        return false;
    }

    const unsigned long now = millis();
    if(automaticRefillPhaseStartedAt == 0)
    {
        automaticRefillPhaseStartedAt = now;
        automaticRefillAttemptStartLevel = sensors.waterLevelCm;
        Serial.println("[REFILL] starting attempt 1");
    }
    const unsigned long elapsed = now - automaticRefillPhaseStartedAt;

    if(automaticRefillPhase == AutomaticRefillPhase::RUNNING)
    {
        // Independent-deadline reconciliation (critical verification report,
        // Priority 4): ActuatorManager's esp_timer deadline may have already
        // force-closed the solenoid - possibly while loop() was stalled
        // inside a blocking Firebase/RTC call - before the elapsed-time
        // check just below ever got a chance to run this tick. Mirrors
        // handleDosingPH()/handleDosingEC()'s own reconciliation: an on-time-
        // or-later cutoff continues into the normal RUNNING->SETTLING
        // transition; an unexpectedly early one routes through the existing
        // failure/abort path instead of being treated as a completed
        // interval.
        if(actuatorManager.getStatus(SOLENOID).forcedOffByDeadline)
        {
            actuatorManager.requestCommand(
                SOLENOID, false, "automatic", now, 100, "refill", "independent deadline");

            if(elapsed >= AUTOMATIC_REFILL_RUN_TIME)
            {
                automaticRefillPhase = AutomaticRefillPhase::SETTLING;
                automaticRefillPhaseStartedAt = now;
                Serial.print("[REFILL] attempt ");
                Serial.print(automaticRefillAttempt);
                Serial.println(" run interval complete (independent deadline); settling");
            }
            else
            {
                failCurrentSubsystem("Automatic refill stopped unexpectedly early by independent safety deadline.");
            }
            return true;
        }

        if(elapsed < AUTOMATIC_REFILL_RUN_TIME)
        {
            return false;
        }

        actuatorManager.requestCommand(
            SOLENOID, false, "automatic", now, 100, "refill", "bounded refill interval complete");
        automaticRefillPhase = AutomaticRefillPhase::SETTLING;
        automaticRefillPhaseStartedAt = now;

        Serial.print("[REFILL] attempt ");
        Serial.print(automaticRefillAttempt);
        Serial.println(" run interval complete; settling");
        return true;
    }

    actuatorManager.requestCommand(
        SOLENOID, false, "automatic", now, 100, "refill", "settling before refill evaluation");

    if(elapsed < AUTOMATIC_REFILL_SETTLE_TIME)
    {
        return true;
    }

    Serial.print("[REFILL] attempt ");
    Serial.print(automaticRefillAttempt);
    Serial.print(" start=");
    Serial.print(automaticRefillAttemptStartLevel, 2);
    Serial.print(" settled=");
    Serial.println(sensors.waterLevelCm, 2);

    // Same refillStopConfirmed requirement as the continuous handleRefilling()
    // path above (resilience pass follow-up) - the preceding
    // AUTOMATIC_REFILL_SETTLE_TIME wait already gives it ample read cycles to
    // resolve, so this adds no additional delay in practice.
    if(sensors.refillStopConfirmed)
    {
        Serial.print("[REFILL] depth=");
        Serial.print(sensors.waterLevelCm, 2);
        Serial.print("cm stopThreshold=");
        Serial.print(systemState.refillStopLevelCm, 2);
        Serial.println("cm -> COMPLETE");
        completeRefillSuccess();
        return true;
    }

    // Reached only with a VALID, trustworthy water-level reading that has
    // genuinely settled and is still short of refillStopLevelCm after a real
    // run+settle cycle - see handleRefilling()'s own matching comment on the
    // canRefill() check above it, which is the SEPARATE fail-fast path for
    // an untrustworthy measurement/safety condition and never reaches here
    // at all. This is "valid sensor, insufficient fill progress" - the only
    // case the 3-attempt budget is intentionally spent on.
    if(automaticRefillAttempt >= MAX_REFILL_ATTEMPTS)
    {
        Serial.println("[REFILL] maximum automatic refill attempts reached");
        failCurrentSubsystem("Automatic refill failed after 3 bounded attempts.");
        Serial.print("[REFILL-LOCK] SET locked=true reason=max_attempts depth=");
        Serial.println(sensors.waterLevelCm, 2);
        return true;
    }

    automaticRefillAttempt++;
    automaticRefillPhase = AutomaticRefillPhase::RUNNING;
    automaticRefillPhaseStartedAt = now;
    automaticRefillAttemptStartLevel = sensors.waterLevelCm;

    Serial.print("[REFILL] starting attempt ");
    Serial.println(automaticRefillAttempt);
    return true;
}

void AutomationManager::handleRefilling()
{
    // Developer testing override: an AUTOMATIC refill (low-water triggered)
    // is exited cleanly the instant the override is on, without touching a
    // manual "Trigger Refill" request (RequestSource::MANUAL is left
    // completely alone here) and without routing through
    // abortCurrentOperation()/failCurrentSubsystem(), which would latch
    // refillSubsystemLocked - a real safety-fault flag this benign bypass
    // must never set. Re-checked every tick (not just on the flag's rising
    // edge), so it equally covers a refill that was already running when the
    // override turned on.
    if(systemState.ignoreWaterLevelAutomation &&
       systemState.operationRequest.source == RequestSource::AUTOMATIC)
    {
        if(!devWaterOverrideExitLogged)
        {
            Serial.println("[DEV WATER] exiting automatic REFILLING due to developer override");
            devWaterOverrideExitLogged = true;
        }

        actuatorManager.requestCommand(
            SOLENOID, false, "automatic", millis(), 100, "refill",
            "Developer override: automatic water-level refill bypassed");

        systemState.reservoirLocked = false;

        failCurrentOperation(
            "Developer override: automatic water-level refill bypassed");

        changeState(STARTUP);

        return;
    }
    devWaterOverrideExitLogged = false;

    // Any refill actually in progress supersedes a prior manual acceptance,
    // regardless of which path started it (low-water auto-trigger, or the
    // admin's "Start Reservoir Refill" button).
    manualRefillAcceptedLevel = NAN;

    alertManager.update();

    SafetyResult result =
        safetyManager.canRefill();

    // Intentional fail-fast distinction (reservoir/refill lifecycle audit,
    // confirmed by design - not a bug): this SafetyResult check, re-evaluated
    // every tick, is a SEPARATE failure path from handleBoundedAutomaticRefill()'s
    // own MAX_REFILL_ATTEMPTS counter below. canRefill() returning non-SAFE
    // here means the water-level MEASUREMENT ITSELF is untrustworthy
    // (SENSOR_FAULT - validWaterLevel()'s own 3-tick debounce already ruled
    // out a single bad reading) or a genuine safety/subsystem lock is active
    // - an UNRELIABLE INPUT, not "the solenoid ran and the level still isn't
    // there yet". It locks immediately via abortCurrentOperation() ->
    // failCurrentSubsystem(), on attempt 1 if that is when it happens,
    // WITHOUT consuming or depending on automaticRefillAttempt at all. A
    // valid, trustworthy reading that simply hasn't reached the stop level
    // yet is the ONLY case that consumes the bounded 3-attempt budget - see
    // handleBoundedAutomaticRefill()'s own matching comment. Do not conflate
    // the two: a flaky sensor must never be retried blindly against a
    // solenoid, but slow physical fill progress should be.
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
        fabsf(sensors.waterLevelCm - lastRefillWaterLevel) > 0.01f ||
        fabsf(systemState.refillStartLevelCm - lastRefillStartLevel) > 0.01f ||
        fabsf(systemState.refillStopLevelCm - lastRefillStopLevel) > 0.01f;

    if(diagnosticsChanged)
    {
        if (debugManager.shouldPrintDebug(DebugCategory::WATER))
        {
            Serial.print("[REFILL] source=");
            Serial.println(mockSource ? "MOCK" : "PHYSICAL");
            Serial.print("[REFILL] depth=");
            Serial.println(sensors.waterLevelCm, 2);
            Serial.print("[REFILL] refillStartLevelCm=");
            Serial.println(systemState.refillStartLevelCm, 2);
            Serial.print("[REFILL] refillStopLevelCm=");
            Serial.println(systemState.refillStopLevelCm, 2);
        }

        refillDiagnosticsInitialized = true;
        lastRefillMockSource = mockSource;
        lastRefillWaterLevel = sensors.waterLevelCm;
        lastRefillStartLevel = systemState.refillStartLevelCm;
        lastRefillStopLevel = systemState.refillStopLevelCm;
    }

    // An automatic refill is evaluated exclusively by
    // handleBoundedAutomaticRefill()'s run+settle cycle - see that
    // function's matching gate. Without this guard, the plain "already
    // at/above stop level" shortcut below fires the instant waterLevelCm
    // crosses the threshold, even mid-run, completing the refill
    // immediately instead of waiting out the run + settle window and
    // evaluating once per attempt. Manual refill requests are excluded,
    // same as handleBoundedAutomaticRefill()'s own gate.
    const bool boundedRefillActive =
        systemState.operationRequest.source == RequestSource::AUTOMATIC;

    // refillStopConfirmed (resilience pass follow-up) requires
    // WATER_LEVEL_STEP_CONFIRM_COUNT consecutive ACCEPTED readings at/above
    // refillStopLevelCm - see SensorManager::readWaterLevel(). Not a plain
    // sensors.waterLevelCm >= refillStopLevelCm comparison any more: that
    // alone let one small transient reading falsely complete a refill.
    if(!boundedRefillActive &&
       sensors.refillStopConfirmed)
    {
        if (debugManager.shouldPrintDebug(DebugCategory::WATER))
        {
            Serial.print("[REFILL] depth=");
            Serial.print(sensors.waterLevelCm, 2);
            Serial.print("cm stopThreshold=");
            Serial.print(systemState.refillStopLevelCm, 2);
            Serial.println("cm -> COMPLETE");
        }
        completeRefillSuccess();
        return;
    }

    if(handleBoundedAutomaticRefill())
    {
        return;
    }

    actuatorManager.requestCommand(
        SOLENOID, true, "automatic", millis(), 100, "refill");
}

void AutomationManager::stopRefillManually()
{
    if(systemState.currentMode != REFILLING)
    {
        return;
    }

    Serial.print("[REFILL] Manually stopped - admin accepted current water depth=");
    Serial.println(sensors.waterLevelCm, 2);

    // See the member's own comment: without this, the low-water trigger
    // below (still true, since refillStartLevelCm hasn't itself changed)
    // would just re-open the solenoid again on the very next check.
    manualRefillAcceptedLevel = sensors.waterLevelCm;

    actuatorManager.requestCommand(
        SOLENOID, false, "automatic", millis(), 100, "refill",
        "Manually stopped - admin accepted current water level");

    systemState.reservoirLocked = false;

    if(systemState.automationTestSubsystem == AutomationTestSubsystem::REFILL)
    {
        Serial.println("[AUTO-TEST] manual Solenoid override interrupted refill");
    }

    completeCurrentOperation();

    if(systemState.automationTestSubsystem == AutomationTestSubsystem::REFILL)
    {
        // Isolated REFILL test: same isolation requirement as
        // completeRefillSuccess() - a manual stop must not hand off to
        // STARTUP's fog sequence either. Land in NORMAL, where
        // handleNormal()'s automationAllowed() gates keep every unrelated
        // controller paused for as long as REFILL test mode stays selected.
        changeState(NORMAL);
        return;
    }

    changeState(STARTUP);
}

// See manualRefillAcceptedLevel's own comment for the scenario this exists
// for. isnan() means no acceptance is in effect - ordinary threshold rules
// apply unmodified. Centimeters, same as manualRefillAcceptedLevel itself.
bool AutomationManager::shouldAutoRefill() const
{
    if(isnan(manualRefillAcceptedLevel))
    {
        return true;
    }

    return sensors.waterLevelCm < manualRefillAcceptedLevel;
}

// See this function's own declaration in AutomationManager.h for the full
// reasoning. alertState.lowWater alone is a debounced ALERT (sufficient to
// notify a user, and it stays true across arbitrarily many main-loop ticks
// off a single accepted reading) - sensors.refillStartConfirmed is the
// STRICTER, separate confirmation (3 consecutive ACCEPTED HC-SR04 readings,
// SensorManager::readWaterLevel()) that must additionally hold before
// automation is trusted to actually open the solenoid. shouldAutoRefill()
// is the existing manual-acceptance "snooze" check (manualRefillAcceptedLevel),
// unchanged and still composed in here exactly as both trigger sites already
// required it.
bool AutomationManager::autoRefillEligible() const
{
    return alertState.lowWater &&
        sensors.refillStartConfirmed &&
        shouldAutoRefill();
}

//==================================================
// Serial Monitor Focus Mode - compact dependency summaries
//==================================================
// One line per controller, printed only when the isolated controller's
// relevant inputs actually change (edge-triggered via inputValueChanged(),
// never every tick) - diagnostics only, read nothing back into any
// automation/safety decision.

void AutomationManager::logPHInputSummary()
{
    if (!debugManager.shouldPrintDebug(DebugCategory::PH)) return;

    static bool initialized = false;
    static float lastPh = NAN;
    static bool lastStable = false;
    static float lastWater = NAN;
    static bool lastWaterValid = false;

    const bool stable = sensorManager.isPhCurrentlyStable();
    const bool waterValid = isfinite(sensors.waterLevelCm);

    if (initialized &&
        !inputValueChanged(sensors.ph, lastPh) &&
        stable == lastStable &&
        !inputValueChanged(sensors.waterLevelCm, lastWater) &&
        waterValid == lastWaterValid)
    {
        return;
    }

    initialized = true;
    lastPh = sensors.ph;
    lastStable = stable;
    lastWater = sensors.waterLevelCm;
    lastWaterValid = waterValid;

    Serial.print("[PH-INPUT] pH=");
    Serial.print(sensors.ph, 2);
    Serial.print(" stable=");
    Serial.print(stable ? "true" : "false");
    Serial.print(" water=");
    Serial.print(sensors.waterLevelCm, 2);
    Serial.print(" waterValid=");
    Serial.println(waterValid ? "true" : "false");
}

void AutomationManager::logECInputSummary()
{
    if (!debugManager.shouldPrintDebug(DebugCategory::EC)) return;

    static bool initialized = false;
    static float lastEc = NAN;
    static bool lastStable = false;
    static float lastWater = NAN;
    static bool lastWaterValid = false;

    const bool stable = sensorManager.isEcCurrentlyStable();
    const bool waterValid = isfinite(sensors.waterLevelCm);

    if (initialized &&
        !inputValueChanged(sensors.ec, lastEc) &&
        stable == lastStable &&
        !inputValueChanged(sensors.waterLevelCm, lastWater) &&
        waterValid == lastWaterValid)
    {
        return;
    }

    initialized = true;
    lastEc = sensors.ec;
    lastStable = stable;
    lastWater = sensors.waterLevelCm;
    lastWaterValid = waterValid;

    Serial.print("[EC-INPUT] ec=");
    Serial.print(sensors.ec, 2);
    Serial.print(" stable=");
    Serial.print(stable ? "true" : "false");
    Serial.print(" water=");
    Serial.print(sensors.waterLevelCm, 2);
    Serial.print(" waterValid=");
    Serial.println(waterValid ? "true" : "false");
}

void AutomationManager::logCoolingInputSummary()
{
    if (!debugManager.shouldPrintDebug(DebugCategory::COOLING)) return;

    static bool initialized = false;
    static float lastWaterTemp = NAN;
    static float lastWater = NAN;
    static bool lastWaterValid = false;

    const bool waterValid = isfinite(sensors.waterLevelCm);

    if (initialized &&
        !inputValueChanged(sensors.waterTemp, lastWaterTemp) &&
        !inputValueChanged(sensors.waterLevelCm, lastWater) &&
        waterValid == lastWaterValid)
    {
        return;
    }

    initialized = true;
    lastWaterTemp = sensors.waterTemp;
    lastWater = sensors.waterLevelCm;
    lastWaterValid = waterValid;

    Serial.print("[COOLING-INPUT] waterTemp=");
    Serial.print(sensors.waterTemp, 2);
    Serial.print(" water=");
    Serial.print(sensors.waterLevelCm, 2);
    Serial.print(" waterValid=");
    Serial.println(waterValid ? "true" : "false");
}

void AutomationManager::logFogInputSummary(const char* cadenceLabel)
{
    if (!debugManager.shouldPrintDebug(DebugCategory::FOGGING)) return;

    static bool initialized = false;
    static float lastWater = NAN;
    static float lastPh = NAN;
    static bool lastPhStable = false;
    static float lastEc = NAN;
    static bool lastEcStable = false;
    static bool lastDhtAvailable = false;
    static bool lastDhtStale = false;
    static const char* lastCadence = "";

    const bool waterValid = isfinite(sensors.waterLevelCm);
    const bool phStable = sensorManager.isPhCurrentlyStable();
    const bool ecStable = sensorManager.isEcCurrentlyStable();

    const bool changed = !initialized ||
        inputValueChanged(sensors.waterLevelCm, lastWater) ||
        inputValueChanged(sensors.ph, lastPh) ||
        phStable != lastPhStable ||
        inputValueChanged(sensors.ec, lastEc) ||
        ecStable != lastEcStable ||
        sensors.dhtAvailable != lastDhtAvailable ||
        sensors.dhtStale != lastDhtStale ||
        strcmp(cadenceLabel, lastCadence) != 0;

    if (!changed) return;

    initialized = true;
    lastWater = sensors.waterLevelCm;
    lastPh = sensors.ph;
    lastPhStable = phStable;
    lastEc = sensors.ec;
    lastEcStable = ecStable;
    lastDhtAvailable = sensors.dhtAvailable;
    lastDhtStale = sensors.dhtStale;
    lastCadence = cadenceLabel;

    Serial.print("[FOG-INPUT] water=");
    Serial.print(sensors.waterLevelCm, 2);
    Serial.print("cm waterValid=");
    Serial.print(waterValid ? "true" : "false");
    Serial.print(" pH=");
    Serial.print(sensors.ph, 2);
    Serial.print(" stable=");
    Serial.print(phStable ? "true" : "false");
    Serial.print(" EC=");
    Serial.print(sensors.ec, 2);
    Serial.print(" stable=");
    Serial.print(ecStable ? "true" : "false");
    Serial.print(" dht=");
    Serial.print(sensors.dhtAvailable ? "OK" : (sensors.dhtStale ? "STALE" : "UNAVAILABLE"));
    if (sensors.dhtAvailable)
    {
        Serial.print(" airTemp=");
        Serial.print(sensors.temperature, 1);
    }
    Serial.print(" cadence=");
    Serial.println(cadenceLabel);
}

void AutomationManager::logPHDecisionLine(const String& line)
{
    static String lastLine = "";
    if (line == lastLine) return;
    lastLine = line;
    Serial.println(line);
}

void AutomationManager::logECDecisionLine(const String& line)
{
    static String lastLine = "";
    if (line == lastLine) return;
    lastLine = line;
    Serial.println(line);
}

//handle ph dosing
void AutomationManager::handleDosingPH()
{
    alertManager.update();
    logPHInputSummary();

    SafetyResult result =
        safetyManager.canDosePH();

    if(result != SafetyResult::SAFE)
    {
        abortCurrentOperation(result);
        return;
    }

    // Independent-deadline reconciliation (critical verification report,
    // Priority 3): ActuatorManager's esp_timer deadline may have already
    // force-stopped the active pump - possibly while loop() was stalled
    // inside a blocking Firebase/RTC call - before the dose-duration check
    // below ever got a chance to run this tick. Check only the pump this
    // dose is actually driving, not both, so a stale flag left on the
    // opposite pump from an earlier, different-direction dose can never be
    // misread as this one's own (ActuatorManager clears the flag on every
    // fresh command, but this keeps the read itself unambiguous either way).
    const Actuator activePhPump =
        systemState.phDirection == PH_UP ? PH_UP_PUMP : PH_DOWN_PUMP;
    if(actuatorManager.getStatus(activePhPump).forcedOffByDeadline)
    {
        actuatorManager.requestCommand(PH_UP_PUMP, false, "automatic", millis());
        actuatorManager.requestCommand(PH_DOWN_PUMP, false, "automatic", millis());

        if(millis() - systemState.stateStartTime >= systemState.phDoseTime)
        {
            // Fired at/after the dose's own intended duration - loop()
            // simply could not get back in time to apply the normal on-time
            // stop below. Treat exactly like that normal stop, not a fault.
            Serial.println("[PH] Independent deadline confirmed dose end (loop delayed); continuing to stabilization");
            changeState(STABILIZING_PH);
        }
        else
        {
            // Fired unexpectedly early relative to the dose's own intended
            // duration - do not pretend the dose completed successfully;
            // route through the existing failure/abort path instead.
            failCurrentSubsystem("pH dose stopped unexpectedly early by independent safety deadline.");
        }
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
    logPHInputSummary();

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

    // Measured from circulation actually being confirmed running (see
    // phStabilizationCirculationConfirmedAt's own comment), not from
    // stateStartTime: entering this state and CIRCULATION_PUMP reaching
    // confirmed RUNNING are not the same tick, so anchoring on state entry
    // under-counted actual pump-on time by however long that ramp-up took.
    // Still 0 (updateCooling() hasn't confirmed circulation yet this
    // episode) means the wait has not started.
    if(phStabilizationCirculationConfirmedAt != 0 &&
       millis() - phStabilizationCirculationConfirmedAt >=
       PH_STABILIZATION_TIME)
    {
        // Past the initial silent settle window and purely watching now -
        // let fogging resume (SafetyManager::canFog() only allows it once
        // this is true AND pH is confirmed within [minPH, maxPH], which
        // canFog() checks independently). Mirrors handleNormal()'s own
        // fogControllerAllowed/validateNormalOperation()/processFogCycle()
        // sequence - reusing validateNormalOperation() here re-derives
        // canFog() (and therefore this same watch-phase/range check) fresh
        // every tick, so fogging still stops immediately if pH drifts back
        // out of range or a redose starts.
        systemState.phWatchPhaseActive = true;

        const bool fogControllerAllowed =
            automationAllowed(AutomationTestSubsystem::FOGGING);
        if(fogControllerAllowed && validateNormalOperation())
        {
            processFogCycle();
        }

        // First checkpoint: one-shot publish of whatever value exists once
        // the initial silent window (30s circulation + 60s silent read,
        // PH_STABILIZATION_TIME) has passed, regardless of whether it has
        // settled yet - quiet-monitoring/4-minute-budget redesign.
        if(!systemState.phFirstCheckpointPublished)
        {
            systemState.phPublishPending = true;
            systemState.phFirstCheckpointPublished = true;
        }

        const bool budgetExpired =
            millis() - systemState.correctionCycleStartAt >=
            PH_EC_CORRECTION_STALL_TIMEOUT_MS;

        // Stable-hold-for-publish bookkeeping: tracks how long the reading
        // has sat continuously inside SensorManager's own stability window,
        // independent of the trend re-sample below - this is what
        // eventually redoses a genuinely stalled-but-out-of-range plateau,
        // or completes the correction, once it has held long enough to
        // trust (PH_EC_STABLE_HOLD_FOR_PUBLISH_MS).
        if(sensorManager.isPhCurrentlyStable())
        {
            if(systemState.phStableSince == 0)
            {
                systemState.phStableSince = millis();
            }
        }
        else
        {
            systemState.phStableSince = 0;
            systemState.phStableCheckpointPublished = false;
        }

        if(systemState.phStableSince != 0 &&
           !systemState.phStableCheckpointPublished &&
           millis() - systemState.phStableSince >=
           PH_EC_STABLE_HOLD_FOR_PUBLISH_MS)
        {
            systemState.phPublishPending = true;
            systemState.phStableCheckpointPublished = true;

            // Do not decide retry-vs-complete from the pre-dose/pre-disturbance
            // value sensors.ph is still (correctly) retaining for
            // Firebase/display - wait here until the live pH signal has
            // reconfirmed a fresh stable reading. Bounded by the existing
            // PH_EC_STABLE_TIMEOUT_MS -> SENSOR_FAULT -> canDosePH() path
            // already re-checked every tick above, so a probe that never
            // restabilizes still aborts via the existing safety model
            // rather than waiting forever.
            if(canStartNewPHCorrection())
            {
                const bool targetReached =
                    systemState.phDirection == PH_UP
                        ? sensors.ph >= systemState.phTargetMin
                        : sensors.ph <= systemState.phTargetMax;

                if(targetReached)
                {
                    systemState.phAttempts = 0;
                    systemState.phDirection = PH_NONE;
                    systemState.reservoirLocked = false;
                    systemState.correctionCycleStartAt = 0;

                    completeCurrentOperation();

                    Serial.println("[PH] correction completed");

                    changeState(
                        NORMAL);

                    return;
                }

                if(!budgetExpired)
                {
                    // A confirmed stable-but-out-of-range plateau: the
                    // clearest possible "no further passive movement,
                    // redose now" signal - no need to wait for the next
                    // trend re-sample below.
                    systemState.phAttempts++;

                    // Continue toward the inner target. Only reverse
                    // direction after an actual overshoot beyond the
                    // opposite inner target.
                    if(systemState.phDirection == PH_UP && sensors.ph > systemState.phTargetMax)
                        systemState.phDirection = PH_DOWN;
                    else if(systemState.phDirection == PH_DOWN && sensors.ph < systemState.phTargetMin)
                        systemState.phDirection = PH_UP;

                    systemState.phDoseTime = PH_DOSING_TIME;
                    systemState.firstCorrectionCycle = false;

                    changeState(
                        DOSING_PH);

                    return;
                }
            }
        }

        // Trend re-sample: catches a reversal WHILE the reading is still
        // actively moving, before it ever settles into a stable plateau
        // (the block above only fires once SensorManager's own window
        // confirms no movement for a full stability-window duration).
        if(millis() - systemState.phLastTrendCheckAt >= PH_EC_RECHECK_INTERVAL_MS)
        {
            systemState.phLastTrendCheckAt = millis();

            if(isnan(systemState.phTrendReferenceValue))
            {
                systemState.phTrendReferenceValue = sensors.ph;
            }
            else
            {
                auto distanceToTarget = [](float ph, float targetMin, float targetMax) -> float
                {
                    if(ph < targetMin) return targetMin - ph;
                    if(ph > targetMax) return ph - targetMax;
                    return 0.0f;
                };

                const float previousDistance = distanceToTarget(
                    systemState.phTrendReferenceValue, systemState.phTargetMin, systemState.phTargetMax);
                const float currentDistance = distanceToTarget(
                    sensors.ph, systemState.phTargetMin, systemState.phTargetMax);

                if(previousDistance - currentDistance > PH_TREND_NOISE_FLOOR)
                {
                    systemState.phLastTrendImproving = true;
                }
                else if(currentDistance - previousDistance > PH_TREND_NOISE_FLOOR)
                {
                    // Reversal - moving away from target rather than
                    // toward it. Dose again right away rather than waiting
                    // for the reading to settle into a stable plateau.
                    systemState.phLastTrendImproving = false;

                    if(!budgetExpired && canStartNewPHCorrection())
                    {
                        systemState.phAttempts++;

                        if(systemState.phDirection == PH_UP && sensors.ph > systemState.phTargetMax)
                            systemState.phDirection = PH_DOWN;
                        else if(systemState.phDirection == PH_DOWN && sensors.ph < systemState.phTargetMin)
                            systemState.phDirection = PH_UP;

                        systemState.phDoseTime = PH_DOSING_TIME;
                        systemState.firstCorrectionCycle = false;

                        systemState.phTrendReferenceValue = sensors.ph;

                        changeState(
                            DOSING_PH);

                        return;
                    }
                }
                else
                {
                    // Within the noise floor: no meaningful movement either
                    // way - treated as no-progress for the budget verdict
                    // below, but not itself worth redosing before the
                    // stable-hold-for-publish path above confirms a genuine
                    // plateau.
                    systemState.phLastTrendImproving = false;
                }

                systemState.phTrendReferenceValue = sensors.ph;
            }
        }

        // Budget-expiry verdict: PH_EC_CORRECTION_STALL_TIMEOUT_MS is a hard
        // upper bound on the whole episode, not merely a threshold for a
        // "stalled" verdict - CONFIRMED BUG FIX (correction-budget limbo):
        // this used to also require !phLastTrendImproving, which let a
        // reading classified as still (even glacially) improving remain
        // parked in STABILIZING_PH indefinitely once the budget expired -
        // dosing correctly stopped (both redose paths above are separately
        // gated on !budgetExpired), but reservoirLocked stayed true and the
        // state never returned to NORMAL, permanently blocking the OTHER
        // chemical subsystem (EC) from ever being evaluated again. The
        // target-reached check earlier in this same block already returns
        // early via completeCurrentOperation() the instant the target IS
        // reached, budget expired or not, so reaching this line already
        // means the target was NOT reached - trend classification has no
        // further bearing on whether the episode terminates, only on
        // whether it redoses WHILE still under budget. alertManager.update()
        // above already re-evaluates phOutOfRange/phHigh/phLow off
        // sensors.ph every tick regardless of anything in this function, so
        // a subsequent out-of-range reading still surfaces through the
        // existing alert/notification path even after this locks -
        // monitoring is unaffected, only automatic dosing stops.
        if(budgetExpired)
        {
            failCurrentSubsystem("Maximum pH correction time reached before the correction target was achieved.");
        }
    }
}

//handle ec dosing
void AutomationManager::handleDosingEC()
{
    alertManager.update();
    logECInputSummary();

    SafetyResult result = systemState.ecDirection == EC_RAISE
        ? safetyManager.canDoseEC()
        : safetyManager.canDiluteEC();

    if(result != SafetyResult::SAFE)
    {
        abortCurrentOperation(result);
        return;
    }

    // Independent-deadline reconciliation (critical verification report,
    // Priority 3): mirrors handleDosingPH()'s own reconciliation above -
    // checks only the actuator(s) this dose direction is actually driving
    // (GROW_PUMP/BLOOM_PUMP for a raise, SOLENOID for a dilution), so a
    // stale flag from an unrelated earlier run is never misread as this
    // one's own.
    const bool ecDeadlineFired = systemState.ecDirection == EC_RAISE
        ? (actuatorManager.getStatus(GROW_PUMP).forcedOffByDeadline ||
           actuatorManager.getStatus(BLOOM_PUMP).forcedOffByDeadline)
        : actuatorManager.getStatus(SOLENOID).forcedOffByDeadline;
    if(ecDeadlineFired)
    {
        actuatorManager.requestCommand(GROW_PUMP, false, "automatic", millis());
        actuatorManager.requestCommand(BLOOM_PUMP, false, "automatic", millis());
        actuatorManager.requestCommand(SOLENOID, false, "automatic", millis(), 100,
            systemState.ecDirection == EC_DILUTE ? "dilution" : "");

        if(millis() - systemState.stateStartTime >= systemState.ecDoseTime)
        {
            // Fired at/after the dose's own intended duration - loop()
            // simply could not get back in time to apply the normal on-time
            // stop below. Treat exactly like that normal stop, not a fault.
            Serial.println("[EC] Independent deadline confirmed dose end (loop delayed); continuing to stabilization");
            systemState.ecDoseTime = 0;
            changeState(STABILIZING_EC);
        }
        else
        {
            // Fired unexpectedly early relative to the dose's own intended
            // duration - do not pretend the dose completed successfully;
            // route through the existing failure/abort path instead.
            failCurrentSubsystem("EC dose stopped unexpectedly early by independent safety deadline.");
        }
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

        // Active dosing has ended - ecDoseTime represents the ACTIVE dosing
        // duration, not a sticky last-used value (see this cleanup's own
        // task): 0 for as long as no dose is actually running. A retry sets
        // it back to EC_DOSING_TIME immediately before its own DOSING_EC
        // re-entry (handleStabilizingEC()), and success/failure both land in
        // NORMAL with it already at this 0.
        systemState.ecDoseTime = 0;

        changeState(STABILIZING_EC);
    }
}

//handle ec stabilization
void AutomationManager::handleStabilizingEC()
{
    alertManager.update();
    logECInputSummary();

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

    // Measured from circulation actually being confirmed running (see
    // ecStabilizationCirculationConfirmedAt's own comment), not from
    // stateStartTime - same reasoning and fix as handleStabilizingPH()'s
    // matching timer. Still 0 (updateCooling() hasn't confirmed circulation
    // yet this episode) means the wait has not started.
    if(ecStabilizationCirculationConfirmedAt != 0 &&
       millis() - ecStabilizationCirculationConfirmedAt >=
       EC_STABILIZATION_TIME)
    {
        alertManager.update();

        // Past the initial silent settle window - let fogging resume. See
        // handleStabilizingPH()'s matching comment.
        systemState.ecWatchPhaseActive = true;

        const bool fogControllerAllowed =
            automationAllowed(AutomationTestSubsystem::FOGGING);
        if(fogControllerAllowed && validateNormalOperation())
        {
            processFogCycle();
        }

        // First checkpoint: one-shot publish - see handleStabilizingPH()'s
        // matching comment.
        if(!systemState.ecFirstCheckpointPublished)
        {
            systemState.ecPublishPending = true;
            systemState.ecFirstCheckpointPublished = true;
        }

        const bool budgetExpired =
            millis() - systemState.correctionCycleStartAt >=
            PH_EC_CORRECTION_STALL_TIMEOUT_MS;

        // Stable-hold-for-publish bookkeeping - see handleStabilizingPH()'s
        // matching comment.
        if(sensorManager.isEcCurrentlyStable())
        {
            if(systemState.ecStableSince == 0)
            {
                systemState.ecStableSince = millis();
            }
        }
        else
        {
            systemState.ecStableSince = 0;
            systemState.ecStableCheckpointPublished = false;
        }

        if(systemState.ecStableSince != 0 &&
           !systemState.ecStableCheckpointPublished &&
           millis() - systemState.ecStableSince >=
           PH_EC_STABLE_HOLD_FOR_PUBLISH_MS)
        {
            systemState.ecPublishPending = true;
            systemState.ecStableCheckpointPublished = true;

            // Do not decide retry-vs-complete from the pre-dose/pre-disturbance
            // value sensors.ec is still (correctly) retaining for
            // Firebase/display - wait here until the live EC signal has
            // reconfirmed a fresh stable reading. Bounded by the existing
            // PH_EC_STABLE_TIMEOUT_MS -> SENSOR_FAULT -> canDoseEC()/
            // canDiluteEC() path already re-checked every tick above, so a
            // probe that never restabilizes still aborts via the existing
            // safety model rather than waiting forever.
            if(canStartNewECCorrection())
            {
                const bool targetReached =
                    systemState.ecDirection == EC_RAISE
                        ? sensors.ec >= systemState.ecTargetMin
                        : sensors.ec <= systemState.ecTargetMax;

                if(targetReached)
                {
                    systemState.ecAttempts = 0;
                    systemState.ecDirection = EC_NONE;
                    systemState.reservoirLocked = false;
                    systemState.correctionCycleStartAt = 0;

                    completeCurrentOperation();

                    Serial.println("[EC] correction completed");

                    changeState(
                        NORMAL);

                    return;
                }

                if(!budgetExpired)
                {
                    // A confirmed stable-but-out-of-range plateau - see
                    // handleStabilizingPH()'s matching comment.
                    systemState.ecAttempts++;

                    systemState.firstCorrectionCycle = false;

                    systemState.ecDoseTime = EC_DOSING_TIME;

                    changeState(
                        DOSING_EC);

                    return;
                }
            }
        }

        // Trend re-sample - see handleStabilizingPH()'s matching comment.
        if(millis() - systemState.ecLastTrendCheckAt >= PH_EC_RECHECK_INTERVAL_MS)
        {
            systemState.ecLastTrendCheckAt = millis();

            if(isnan(systemState.ecTrendReferenceValue))
            {
                systemState.ecTrendReferenceValue = sensors.ec;
            }
            else
            {
                auto distanceToTarget = [](float ec, float targetMin, float targetMax) -> float
                {
                    if(ec < targetMin) return targetMin - ec;
                    if(ec > targetMax) return ec - targetMax;
                    return 0.0f;
                };

                const float previousDistance = distanceToTarget(
                    systemState.ecTrendReferenceValue, systemState.ecTargetMin, systemState.ecTargetMax);
                const float currentDistance = distanceToTarget(
                    sensors.ec, systemState.ecTargetMin, systemState.ecTargetMax);

                if(previousDistance - currentDistance > EC_TREND_NOISE_FLOOR)
                {
                    systemState.ecLastTrendImproving = true;
                }
                else if(currentDistance - previousDistance > EC_TREND_NOISE_FLOOR)
                {
                    // Reversal - dose again right away rather than waiting
                    // for the reading to settle into a stable plateau.
                    systemState.ecLastTrendImproving = false;

                    if(!budgetExpired && canStartNewECCorrection())
                    {
                        systemState.ecAttempts++;

                        systemState.firstCorrectionCycle = false;

                        systemState.ecDoseTime = EC_DOSING_TIME;

                        systemState.ecTrendReferenceValue = sensors.ec;

                        changeState(
                            DOSING_EC);

                        return;
                    }
                }
                else
                {
                    systemState.ecLastTrendImproving = false;
                }

                systemState.ecTrendReferenceValue = sensors.ec;
            }
        }

        // Budget-expiry verdict - see handleStabilizingPH()'s matching
        // comment (same correction-budget-limbo fix: the budget is now a
        // hard episode ceiling regardless of ecLastTrendImproving).
        if(budgetExpired)
        {
            failCurrentSubsystem("Maximum EC correction time reached before the correction target was achieved.");
        }
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
    // Safety rejections are always-allow (see the Serial Monitor Focus Mode
    // report) but only when relevant to the isolated controller - reuses
    // shouldPrintStateTransition's own mode->subsystem mapping since
    // currentMode at this point is always the mode the rejected operation
    // was running in (DOSING_PH/STABILIZING_PH/DOSING_EC/STABILIZING_EC/
    // REFILLING).
    if (debugManager.shouldPrintStateTransition(systemState.currentMode, systemState.currentMode))
    {
        Serial.print("[SAFETY] ");
        Serial.print(getStateName(systemState.currentMode));
        Serial.print(" stopped: ");
        Serial.println(reason);
    }

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
        systemState.phDirection = PH_NONE;
        // Quiet-monitoring/4-minute-budget redesign: this episode is over
        // (locked, pending Reset Safety) - the next independent out-of-range
        // episode gets its own fresh budget.
        systemState.correctionCycleStartAt = 0;
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
        systemState.ecDoseTime = 0;
        // See the pH branch's matching comment above.
        systemState.correctionCycleStartAt = 0;
        Serial.print("[EC-LOCK] set reason=");
        Serial.println(reason);
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
        // safetyLock must be set BEFORE turnOffAll(): its automatic OFF
        // commands must bypass any standing manual hold (priority model:
        // HARD SAFETY BLOCK outranks MANUAL) - see
        // ActuatorManager::requestCommand()'s manual-hold guard, which only
        // lets a genuine safety lock through.
        systemState.safetyLock = true;
        actuatorManager.turnOffAll(reason);
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

    // lastProcessedRequestId's ONLY reader is FirebaseManager::
    // isDuplicateRequest(), which exists solely to stop readCommands() from
    // reprocessing the SAME still-present /commands/current document twice -
    // that path is exclusively manual (app REFILL/RESET_SAFETY/pH-EC trigger
    // buttons; see readCommands()'s own comment). An AUTOMATIC-sourced
    // request was never read from that inbox, so it must not advance this
    // watermark: doing so "forgets" the last manual command was already
    // handled, and since that command's node is never deleted (only
    // deduped), the next readCommands() poll after the operation lifecycle
    // returns to IDLE would silently replay it as if it were new - this is
    // exactly how a stale Reset Safety click was being re-consumed and
    // clearing refillSubsystemLocked with water still low.
    if(source == RequestSource::MANUAL)
    {
        systemState.lastProcessedRequestId =
            requestId;
    }
}
//Canopy Climate Control
void AutomationManager::handleCanopyClimate()
{
    float temp = sensors.temperature;
    float humidity = sensors.humidity;

    uint8_t speed;

    // DHT unavailable/stale (see the automation resilience pass report):
    // retain the last AUTOMATIC canopy demand rather than forcing 100% -
    // a stale reading is no basis to change ownership, and PWM command
    // architecture is otherwise unchanged. sensors.dhtAvailable, not
    // isnan(temp)/isnan(humidity) - those now hold a last-good value rather
    // than going NaN, so NaN alone no longer signals unavailability.
    const bool dbgCanopy = debugManager.shouldPrintDebug(DebugCategory::CANOPY);
    static const char* lastLoggedCanopyRule = "";
    const char* canopyRule;

    if (!sensors.dhtAvailable)
    {
        speed = lastAutomaticCanopySpeed;

        // Distinguishes a genuine post-valid staleness hold from never
        // having had a valid DHT reading since boot (temperature is still
        // NaN in that case - see SensorData's own comment) - both use the
        // same retained-speed logic, only the diagnostic label differs.
        canopyRule = isfinite(sensors.temperature) ? "DHT STALE HOLD" : "BOOT FALLBACK";

        static unsigned long lastCanopyDhtLogAt = 0;
        const unsigned long now = millis();
        if (dbgCanopy &&
            (lastCanopyDhtLogAt == 0 || now - lastCanopyDhtLogAt >= AUTO_TEST_BLOCK_LOG_INTERVAL_MS))
        {
            lastCanopyDhtLogAt = now;
            Serial.print("[CANOPY] DHT unavailable -> retaining last automatic speed=");
            Serial.println(speed);
        }
    }
    else
    {
        // 70% is the NORMAL-demand speed, the middle of the 65-75% band
        // real-hardware bench testing (FanPwmSpeedTest.ino, on the actual
        // opto-isolated MOSFET module) found both fans run cleanest in - see
        // CANOPY_BLOWER_PWM_FREQUENCY_HZ's own comment in Config.h.
        speed = 70;

        if (!highAirDemandActive && temp > systemState.highAirTemp)
            highAirDemandActive = true;
        else if (highAirDemandActive && temp <= systemState.airTempRelease)
            highAirDemandActive = false;

        if (!lowAirDemandActive && temp < systemState.lowAirTemp)
            lowAirDemandActive = true;
        else if (lowAirDemandActive && temp >= systemState.coldAirRelease)
            lowAirDemandActive = false;

        if (!highHumidityDemandActive && humidity > systemState.highHumidity)
            highHumidityDemandActive = true;
        else if (highHumidityDemandActive && humidity <= systemState.humidityRelease)
            highHumidityDemandActive = false;

        // Hot/humid (100%) is more protective than cold (50%) and wins if
        // both apply at once (e.g. cold air, high humidity) - see this
        // change's own task note. Plain NORMAL (70%, the default above)
        // applies only when none of the three demands are active.
        if (highAirDemandActive || highHumidityDemandActive) speed = 100;
        else if (lowAirDemandActive) speed = 50;

        canopyRule = (highAirDemandActive || highHumidityDemandActive) ? "HIGH TEMP/HUMIDITY" :
            lowAirDemandActive ? "LOW TEMP" : "NORMAL";

        // Only a fresh DHT-derived decision updates the retained value the
        // unavailable branch above falls back to - see
        // lastAutomaticCanopySpeed's own comment. Both of its former
        // outside-this-function readers are gone: handleCultivationPaused()
        // no longer consumes it (no-active-cultivation-cycle fix - automatic
        // canopy fan is commanded OFF there now, not held at a baseline
        // speed), and processFogCycle()'s root-zone Blower no longer borrows
        // it either (root-blower/canopy-fan speed separation fix - the
        // Blower now uses its own independent systemState.blowerSpeedPercent).
        // This value is CANOPY_FAN-only again, read solely within this
        // function's own DHT-unavailable branch above.
        lastAutomaticCanopySpeed = speed;
    }

    // Serial Monitor Focus Mode: one compact line per RULE change (never
    // every tick) - see DebugManager::shouldPrintDebug()'s own comment.
    if (dbgCanopy && strcmp(canopyRule, lastLoggedCanopyRule) != 0)
    {
        Serial.print("[CANOPY] rule=");
        Serial.print(canopyRule);
        Serial.print(" speed=");
        Serial.println(speed);
        lastLoggedCanopyRule = canopyRule;
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
