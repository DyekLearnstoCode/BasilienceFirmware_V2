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
    // This is deliberately ahead of the cultivation gate: reservoir cooling and
    // the circulation it depends on are hardware protection, not cultivation,
    // and must run whether or not a growth cycle exists.
    updateCooling();

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

    // Ventilation is kept at a safe baseline rather than stopped. No active
    // growth cycle means handleCanopyClimate() itself isn't running, so this
    // mirrors its own DHT-unavailable fallback (see the automation
    // resilience pass report): retain the last automatic canopy demand
    // rather than forcing 100% while DHT is unavailable/stale.
    actuatorManager.requestCommand(
        CANOPY_FAN, true, "automatic", millis(),
        sensors.dhtAvailable ? 50 : lastAutomaticCanopySpeed);
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
    if (automationAllowed(AutomationTestSubsystem::REFILL) &&
        alertState.lowWater && shouldAutoRefill() &&
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
    }

    if(newMode == STABILIZING_EC)
    {
        // See ecStabilizationCirculationConfirmedAt's own comment.
        ecStabilizationCirculationConfirmedAt = 0;
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

            // Startup uses the same experimentally configured fog-
            // distribution airflow as normal automatic fogging (real-
            // hardware Canopy/Blower PWM follow-up) - previously a separate
            // hardcoded 100%, now the one shared systemState.blowerSpeedPercent
            // value used by processFogCycle()'s ON phase too.
            actuatorManager.requestCommand(BLOWER, true, "automatic", millis(), systemState.blowerSpeedPercent, "startup");

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

    // Check for automatic refill before processing manual requests. The
    // developer water-level override (systemState.ignoreWaterLevelAutomation)
    // only ever suppresses this AUTOMATIC trigger - processRefillRequest()
    // below still services an explicit manual "Trigger Refill" unconditionally.
    // refillSubsystemLocked is checked explicitly here, not only inside
    // canRefill() below, so a persistent low-water condition that already
    // exhausted MAX_REFILL_ATTEMPTS can never start a brand-new bounded
    // refill operation (a fresh "starting attempt 1") while that same
    // unresolved episode continues - only the re-arm check above, or an
    // explicit admin reset, can lift it.
    // refillStartConfirmed (resilience pass follow-up, part 2 of the last
    // targeted check) requires WATER_LEVEL_STEP_CONFIRM_COUNT consecutive
    // ACCEPTED readings at/below refillStartLevelCm - see
    // SensorManager::readWaterLevel(). alertState.lowWater is left as an
    // additional gate, unchanged, rather than folded into this condition:
    // its own notification-facing debounce/semantics are untouched, this
    // only adds a stricter, independent requirement before an AUTOMATIC
    // refill is actually allowed to start.
    if (automationAllowed(AutomationTestSubsystem::REFILL) &&
        !systemState.refillSubsystemLocked &&
        alertState.lowWater && sensors.refillStartConfirmed && shouldAutoRefill() &&
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

    if(processRefillRequest())
    {
        return;
    }

    // Automatic re-arm for a terminal PH failure latch, mirroring the
    // refillSubsystemLocked re-arm above: phSubsystemLocked only ever
    // reflects MAX_PH_ATTEMPTS exhaustion for the LAST out-of-range episode
    // (set in failCurrentSubsystem()'s PH branch), not that pH is
    // permanently unsafe. Once pH itself genuinely recovers inside
    // [minPH, maxPH], the condition the lock was raised for is gone, so
    // clear it here - event-driven off the real reading, never a timer -
    // together with phAttempts, so a later, independent out-of-range
    // episode (in either direction) gets its own full MAX_PH_ATTEMPTS run
    // from Attempt 1. Deliberately NOT the mere fact that phDirection would
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
    // ecSubsystemLocked reflects either MAX_EC_ATTEMPTS exhaustion or a
    // RESERVOIR_FULL dilution block (both set it in failCurrentSubsystem()'s
    // EC branch / processECCorrection()'s dilution-blocked branch), neither
    // of which means EC is permanently unsafe. Once EC itself genuinely
    // recovers inside [minEC, maxEC], clear it here - event-driven off the
    // real reading, never a timer - together with ecAttempts, so a later,
    // independent out-of-range episode (either direction) gets its own full
    // MAX_EC_ATTEMPTS run from Attempt 1. Deliberately NOT the mere fact
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
    static bool coolingFoggingSuspendedLogged = false;

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
    logCoolingInputSummary();

    const SafetyResult coolingSafety = safetyManager.canCool();

    const bool automaticCoolingAllowed =
        automationAllowed(AutomationTestSubsystem::COOLING);

    // The app's editable "Water Temperature" maximum (systemState.maxWaterTemp
    // - already validated finite/in-bounds by FirebaseManager::
    // applyTargetRange(), which rejects and keeps the last valid value rather
    // than ever admitting NaN/out-of-bounds) is now the single authoritative
    // cooling-ON ceiling, matching the same >maxWaterTemp comparison the
    // water-temperature alert already uses (AlertManager::
    // updateWaterTemperatureAlert()) - exactly at maxWaterTemp does not start
    // a new cooling cycle, same as it does not raise the alert. The release
    // threshold is derived from it with the existing 2.5C hysteresis
    // (WATER_COOLING_HYSTERESIS) rather than reusing minWaterTemp, which is a
    // separate, unrelated setting.
    //
    // highWaterTemp/coolerOffTemp are refreshed here (not removed) so every
    // existing reader - this function's own diagnostic log below, Firebase
    // status publishing, and ActuatorManager's manual-Peltier-override
    // validation - keeps seeing the true effective thresholds without each
    // needing its own edit, while maxWaterTemp remains the only field an
    // admin actually configures.
    systemState.highWaterTemp = systemState.maxWaterTemp;
    systemState.coolerOffTemp = systemState.maxWaterTemp - WATER_COOLING_HYSTERESIS;

    const int8_t temperatureBand =
        sensors.waterTemp > systemState.highWaterTemp ? 1 :
        (sensors.waterTemp <= systemState.coolerOffTemp ? -1 : 0);

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

    uint8_t demandMask = 0;
    if (coolingDemandActive || manualCoolingDemandActive) demandMask |= DEMAND_PELTIER;
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

    if ((coolingDemandActive || manualCoolingDemandActive) && circulationConfirmed)
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
            (coolingDemandActive || manualCoolingDemandActive)
                ? "waiting_for_circulation" : "");
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

        // Configurable automatic Blower speed (real-hardware Canopy/Blower
        // PWM follow-up) - replaces the previous hard-coded 100%. Only this
        // ON case (the fogger/blower pair actively running) uses the
        // configured value; the OFF/purge branch below is a separate,
        // deliberately-untouched mechanism, not "the pair running".
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
    // Confirmed defect (see the RTC report): getHour()/getMinute() only
    // check RTCManager::isConnected(), not hasValidTime() - so a DS3231
    // that lost power still returns whatever it currently reads, and this
    // schedule would silently run against that garbage "current time"
    // instead of the real one. Leave the grow light in its current
    // commanded state rather than guess; matches the same
    // hasValidTime()-guard NotificationManager already uses for its own
    // time-dependent decisions.
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
        return;
    }

    bool lightEnabled =
        isWithinSchedule(
            systemState.lightOnHour,
            systemState.lightOnMinute,
            systemState.lightOffHour,
            systemState.lightOffMinute);

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
        actuatorManager.requestCommand(
            GROW_LIGHT, false, "automatic", millis());
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
    // regardless of which path started it (low-water auto-trigger, the
    // admin's "Start Reservoir Refill" button, or forceRefill).
    manualRefillAcceptedLevel = NAN;

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
        // Do not decide retry-vs-complete from the pre-dose/pre-disturbance
        // value sensors.ph is still (correctly) retaining for
        // Firebase/display - wait here until the live pH signal has
        // reconfirmed a fresh stable reading. Bounded by the existing
        // PH_EC_STABLE_TIMEOUT_MS -> SENSOR_FAULT -> canDosePH() path already
        // re-checked every tick above, so a probe that never restabilizes
        // still aborts via the existing safety model rather than waiting
        // forever.
        if(!canStartNewPHCorrection())
        {
            return;
        }

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

            systemState.phDoseTime = PH_DOSING_TIME;

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
    logECInputSummary();

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

        // Do not decide retry-vs-complete from the pre-dose/pre-disturbance
        // value sensors.ec is still (correctly) retaining for
        // Firebase/display - wait here until the live EC signal has
        // reconfirmed a fresh stable reading. Bounded by the existing
        // PH_EC_STABLE_TIMEOUT_MS -> SENSOR_FAULT -> canDoseEC()/
        // canDiluteEC() path already re-checked every tick above, so a probe
        // that never restabilizes still aborts via the existing safety
        // model rather than waiting forever.
        if(!canStartNewECCorrection())
        {
            return;
        }

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

            systemState.ecDoseTime = EC_DOSING_TIME;

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
        speed = 50;

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

        // Hot/humid (100%) is more protective than cold (30%) and wins if
        // both apply at once (e.g. cold air, high humidity) - see this
        // change's own task note. Plain NORMAL (50%, the default above)
        // applies only when none of the three demands are active.
        if (highAirDemandActive || highHumidityDemandActive) speed = 100;
        else if (lowAirDemandActive) speed = 30;

        canopyRule = (highAirDemandActive || highHumidityDemandActive) ? "HIGH TEMP/HUMIDITY" :
            lowAirDemandActive ? "LOW TEMP" : "NORMAL";

        // Only a fresh DHT-derived decision updates the retained value the
        // unavailable branch above (and handleCultivationPaused()) fall back
        // to - see lastAutomaticCanopySpeed's own comment.
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
