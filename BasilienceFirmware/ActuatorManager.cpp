#include "ActuatorManager.h"

#include <Arduino.h>

#include "Config.h"
#include "Globals.h"

namespace
{
    const char* const WAITING_FOR_CIRCULATION = "Waiting for circulation pump.";

    const uint8_t actuatorPins[ACTUATOR_COUNT] =
        {
            FOGGER_PIN,
            GROW_LIGHT_PIN,

            BLOWER_PIN,
            SOLENOID_PIN,

            GROW_PUMP_PIN,
            BLOOM_PUMP_PIN,
            PH_UP_PUMP_PIN,
            PH_DOWN_PUMP_PIN,

            CANOPY_FAN_PIN,
            PELTIER_PIN,
            CIRCULATION_PUMP_PIN};

    const char* actuatorLogName(Actuator actuator)
    {
        switch (actuator)
        {
            case FOGGER: return "FOGGER";
            case GROW_LIGHT: return "GROW_LIGHT";
            case BLOWER: return "BLOWER";
            case SOLENOID: return "SOLENOID";
            case GROW_PUMP: return "GROW_PUMP";
            case BLOOM_PUMP: return "BLOOM_PUMP";
            case PH_UP_PUMP: return "PH_UP";
            case PH_DOWN_PUMP: return "PH_DOWN";
            case CANOPY_FAN: return "CANOPY_FAN";
            case PELTIER: return "PELTIER";
            case CIRCULATION_PUMP: return "CIRCULATION_PUMP";
            default: return "UNKNOWN";
        }
    }

    const char* actuatorStateLogName(ActuatorCommandState state)
    {
        switch (state)
        {
            case ActuatorCommandState::OFF: return "OFF";
            case ActuatorCommandState::COMMAND_RECEIVED: return "COMMAND_RECEIVED";
            case ActuatorCommandState::VALIDATING: return "VALIDATING";
            case ActuatorCommandState::REJECTED: return "REJECTED";
            case ActuatorCommandState::ACTIVATING: return "ACTIVATING";
            case ActuatorCommandState::RUNNING: return "RUNNING";
            case ActuatorCommandState::STOPPING: return "STOPPING";
            default: return "UNKNOWN";
        }
    }

    bool actuatorStatusDiffers(const ActuatorStatus& left, const ActuatorStatus& right)
    {
        return left.state != right.state ||
            left.running != right.running ||
            left.speed != right.speed ||
            left.startedAt != right.startedAt ||
            left.source != right.source ||
            left.strategy != right.strategy ||
            left.reason != right.reason ||
            left.overrideActive != right.overrideActive;
    }

    bool isManualSource(const String& source)
    {
        return source == "manual";
    }

    bool validPH(float value)
    {
        return isfinite(value) && value >= 0.0f && value <= 14.0f;
    }

    bool validEC(float value)
    {
        return isfinite(value) && value >= 0.0f;
    }

    bool validPercentage(float value)
    {
        return isfinite(value) && value >= 0.0f && value <= 100.0f;
    }

    bool validEnvironment()
    {
        return isfinite(sensors.temperature) &&
            sensors.temperature >= -40.0f && sensors.temperature <= 100.0f &&
            isfinite(sensors.humidity) &&
            sensors.humidity >= 0.0f && sensors.humidity <= 100.0f;
    }

    // Actuators that already carry an explicit hard manual runtime cap in the
    // RUNNING-state watchdog below (MANUAL_PUMP_RUNTIME/OPERATION_TIMEOUT_MS)
    // get an independent esp_timer deadline for that same cap. Grow Light,
    // Canopy Fan, Blower, and Circulation Pump have no such cap by design
    // (latched-until-explicit-OFF) and deliberately get none here either -
    // see the task report's per-actuator evaluation.
    bool isDeadlineProtected(Actuator actuator)
    {
        switch (actuator)
        {
            case PH_UP_PUMP:
            case PH_DOWN_PUMP:
            case GROW_PUMP:
            case BLOOM_PUMP:
            case SOLENOID:
            case PELTIER:
            case FOGGER:
                return true;
            default:
                return false;
        }
    }

    // Mirrors the exact constant the loop-polled watchdog in update() already
    // uses for the same actuator, so the independent deadline and the normal
    // watchdog always agree on the cap - this is a second enforcement path
    // for the same limit, never a second, different limit.
    unsigned long manualDeadlineMs(Actuator actuator)
    {
        switch (actuator)
        {
            case PH_UP_PUMP:
            case PH_DOWN_PUMP:
            case GROW_PUMP:
            case BLOOM_PUMP:
                return MANUAL_PUMP_RUNTIME;
            default:
                // SOLENOID, PELTIER, FOGGER
                return OPERATION_TIMEOUT_MS;
        }
    }
}

uint8_t ActuatorManager::getPin(Actuator actuator) const
{
    return actuatorPins[actuator];
}

void ActuatorManager::begin()
{
    statusDirty = true;

    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        if (i == CANOPY_FAN)
        {
            // ESP32 Core 3.x API
            ledcAttach(getPin((Actuator)i), 5000, 8); // pin, freq, resolution
            ledcWrite(getPin((Actuator)i), 0); // pin, duty
        }
        else
        {
            pinMode(getPin((Actuator)i), OUTPUT);
            digitalWrite(getPin((Actuator)i), LOW);
        }

        actuatorStates[i] = false;

        commands[i].isPending = false;
        statuses[i].state = ActuatorCommandState::OFF;
        statuses[i].running = false;
        manuallyOverridden[i] = false;
        deadlineExpired[i] = false;

        if (isDeadlineProtected((Actuator)i) && deadlineTimers[i] == nullptr)
        {
            esp_timer_create_args_t timerArgs = {};
            timerArgs.callback = &ActuatorManager::onDeadlineExpired;
            timerArgs.arg = reinterpret_cast<void*>(static_cast<intptr_t>(i));
            // Default (task) dispatch, not ESP_TIMER_ISR: onDeadlineExpired()
            // runs in the esp_timer service task, a normal FreeRTOS task
            // context, so plain Arduino calls (digitalWrite) are safe there -
            // this deliberately avoids true-ISR constraints entirely.
            timerArgs.dispatch_method = ESP_TIMER_TASK;
            timerArgs.name = "actDeadline";
            esp_err_t created = esp_timer_create(&timerArgs, &deadlineTimers[i]);
            if (created != ESP_OK)
            {
                Serial.print("[SAFETY] Independent deadline timer create failed for ");
                Serial.print(actuatorLogName((Actuator)i));
                Serial.print(": esp_err=");
                Serial.println((int)created);
            }
        }
    }
}

void ActuatorManager::turnOn(Actuator actuator)
{
    if (actuator != CANOPY_FAN)
    {
        digitalWrite(getPin(actuator), HIGH);
    }
    // For CANOPY_FAN, speed is applied dynamically in update()

    actuatorStates[actuator] = true;
}

void ActuatorManager::turnOff(Actuator actuator)
{
    if (actuator == CANOPY_FAN)
    {
        ledcWrite(getPin(actuator), 0);
    }
    else
    {
        digitalWrite(getPin(actuator), LOW);
    }

    actuatorStates[actuator] = false;
}

void ActuatorManager::toggle(Actuator actuator)
{
    if (actuatorStates[actuator])
    {
        turnOff(actuator);
    }
    else
    {
        turnOn(actuator);
    }
}

bool ActuatorManager::isOn(Actuator actuator) const
{
    return actuatorStates[actuator];
}

void ActuatorManager::turnOffAll(const String& reason)
{
    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        requestCommand((Actuator)i, false, "automatic", millis(), 100, "", reason);
    }
}

void ActuatorManager::requestCommand(Actuator actuator, bool state, const String& source, double timestamp, uint8_t speed, const String& strategy, const String& reason, bool overrideRequested)
{
    // The circulation pump serves several subsystems at once, so a manual ON
    // is always meaningful and needs no threshold check. A manual OFF is the
    // only direction that can be unsafe: automatic cooling and pH/EC
    // stabilization physically depend on the pump running, so an automatic
    // demand outranks a manual stop. The demand mask that drives the pump is
    // the single source of truth for that - see
    // AutomationManager::isCirculationRequired().
    if (actuator == CIRCULATION_PUMP && !state && isManualSource(source))
    {
        if (automationManager.isCirculationRequired())
        {
            const char* why = automationManager.circulationRequirementReason();

            Serial.print("[CIRCULATION] Manual OFF rejected: ");
            Serial.println(why);

            // The pump keeps running and its state is unchanged; only the
            // reason is published so the app can explain the refusal and
            // re-sync its switch from real actuator status.
            statuses[actuator].reason = why;
            statusDirty = true;
            return;
        }
    }

    // Ignore automatic schedule commands when this actuator is manually overridden in manual mode
    if (systemState.manualMode && source == "automatic" && manuallyOverridden[actuator])
    {
        return;
    }

    if (isManualSource(source))
    {
        Serial.print("[MANUAL] ");
        Serial.print(actuatorLogName(actuator));
        Serial.print(" -> ");
        Serial.print(state ? "ON" : "OFF");
        Serial.println(" received");

        // Manual control cannot take ownership of, or stop, an actuator that
        // is currently being controlled by an automatic path.
        if (statuses[actuator].running && statuses[actuator].source != "manual")
        {
            Serial.print("[MANUAL] ignored: actuator is running under ");
            Serial.print(statuses[actuator].source);
            Serial.println(" control");
            return;
        }

        if (actuator == PELTIER && !state)
        {
            automationManager.setManualCoolingDemand(false);
        }

        // A stored/no-op OFF command must not create a manual override that
        // suppresses a later automatic request.
        if (!state && !statuses[actuator].running && statuses[actuator].source != "manual")
        {
            Serial.println("[MANUAL] ignored: actuator is not manually running");
            return;
        }

        if (actuator == CIRCULATION_PUMP && manuallyOverridden[actuator] != state)
        {
            Serial.println(state
                ? "[CIRCULATION] Manual demand added"
                : "[CIRCULATION] Manual demand removed");
        }

        manuallyOverridden[actuator] = state;
    }

    // Repeated automatic OFF reconciliation must not erase the reason retained
    // by a completed safety stop or timeout.
    if (!state && !statuses[actuator].running && statuses[actuator].state == ActuatorCommandState::OFF)
    {
        return;
    }

    commands[actuator].isPending = true;
    commands[actuator].targetState = state;
    commands[actuator].speed = speed;
    commands[actuator].source = source;
    commands[actuator].strategy = strategy;
    commands[actuator].reason = reason;
    commands[actuator].timestamp = timestamp;
    // Only a manual ON command can carry a meaningful override - defensively
    // stripped for every other case so an OFF or an automatic-source command
    // (which never call this with the app's override flag anyway) can never
    // leave a stale override active on the status this becomes.
    commands[actuator].overrideRequested = overrideRequested && state && isManualSource(source);
}

ActuatorStatus ActuatorManager::getStatus(Actuator actuator) const
{
    return statuses[actuator];
}

bool ActuatorManager::isStatusDirty() const
{
    return statusDirty;
}

void ActuatorManager::markStatusSynced()
{
    statusDirty = false;
}

// esp_timer service-task callback - see the header comment on
// deadlineExpired[] for exactly what is and is not safe to do here.
// Deliberately minimal: force the physical pin OFF now (the actual safety
// action - must not wait for loop() to come back from a blocked Firebase
// call), then hand off to update() via one flag for everything else
// (FSM/actuatorStatus/logging reconciliation). No Strings, no Firebase, no
// other ActuatorManager array is touched from here.
void ActuatorManager::onDeadlineExpired(void* arg)
{
    const int index = static_cast<int>(reinterpret_cast<intptr_t>(arg));
    if (index < 0 || index >= ACTUATOR_COUNT) return;

    digitalWrite(actuatorManager.getPin((Actuator)index), LOW);
    actuatorManager.deadlineExpired[index] = true;
}

void ActuatorManager::armDeadline(Actuator actuator, unsigned long durationMs)
{
    if (deadlineTimers[actuator] == nullptr) return;

    // esp_timer_start_once() on an already-running one-shot fails - stopping
    // first is the documented safe re-arm sequence, and is a harmless no-op
    // if the timer isn't currently running (already fired or never started).
    esp_timer_stop(deadlineTimers[actuator]);
    deadlineExpired[actuator] = false;
    esp_timer_start_once(deadlineTimers[actuator], (uint64_t)durationMs * 1000ULL);
}

void ActuatorManager::disarmDeadline(Actuator actuator)
{
    if (deadlineTimers[actuator] == nullptr) return;
    // Harmless no-op if not currently running (already fired, or this
    // actuator was never armed for the run that's ending).
    esp_timer_stop(deadlineTimers[actuator]);
}

bool ActuatorManager::validateCommand(Actuator actuator, bool targetState, String& outReason, bool runningValidation)
{
    if (!targetState) return true;

    const bool manual = isManualSource(statuses[actuator].source);

    if (manual && !systemState.manualMode)
    {
        outReason = "Manual mode is not enabled.";
        return false;
    }

    if (systemState.safetyLock)
    {
        outReason = "System locked. Emergency stop has been activated.";
        return false;
    }

    switch (actuator)
    {
        case PH_UP_PUMP:
        case PH_DOWN_PUMP:
        case GROW_PUMP:
        case BLOOM_PUMP:
        {
            if ((actuator == PH_UP_PUMP || actuator == PH_DOWN_PUMP) &&
                systemState.phSubsystemLocked)
            {
                outReason = "pH correction subsystem is locked.";
                return false;
            }
            if ((actuator == GROW_PUMP || actuator == BLOOM_PUMP) &&
                systemState.ecSubsystemLocked)
            {
                outReason = "EC correction subsystem is locked.";
                return false;
            }
            const bool phCorrectionOwnsLock =
                (actuator == PH_UP_PUMP || actuator == PH_DOWN_PUMP) &&
                (systemState.currentMode == DOSING_PH || systemState.currentMode == STABILIZING_PH);
            const bool ecCorrectionOwnsLock =
                (actuator == GROW_PUMP || actuator == BLOOM_PUMP) &&
                (systemState.currentMode == DOSING_EC || systemState.currentMode == STABILIZING_EC);

            if ((systemState.reservoirLocked && !phCorrectionOwnsLock && !ecCorrectionOwnsLock) || isOn(SOLENOID))
            {
                outReason = "Reservoir is currently locked by another operation.";
                return false;
            }
            if (!validPercentage(sensors.waterLevel))
            {
                outReason = "Cannot dose: Water-level reading is invalid.";
                return false;
            }
            if (sensors.waterLevel < systemState.refillStartLevel)
            {
                outReason = "Cannot dose: Water reservoir level is too low.";
                return false;
            }

            if ((actuator == PH_UP_PUMP || actuator == PH_DOWN_PUMP) && !validPH(sensors.ph))
            {
                outReason = "Cannot dose: Current pH reading is invalid.";
                return false;
            }
            if ((actuator == GROW_PUMP || actuator == BLOOM_PUMP) && !validEC(sensors.ec))
            {
                outReason = "Cannot dose: Current EC reading is invalid.";
                return false;
            }

            if (actuator == PH_UP_PUMP && isOn(PH_DOWN_PUMP))
            {
                outReason = "Cannot activate pH Up. pH Down is currently running.";
                return false;
            }
            if (actuator == PH_DOWN_PUMP && isOn(PH_UP_PUMP))
            {
                outReason = "Cannot activate pH Down. pH Up is currently running.";
                return false;
            }

            if (manual && (actuator == PH_UP_PUMP || actuator == PH_DOWN_PUMP))
            {
                if (!validPH(sensors.ph))
                {
                    outReason = "Cannot dose: Current pH reading is invalid.";
                    return false;
                }

                // SOFT rule ("pH already inside the configured target range") -
                // the only check in this block an explicit manual override may
                // bypass. Bypassing it also suppresses the runningValidation
                // early-stop-on-target-reached signal for this command, so an
                // overridden dose simply runs for its existing bounded
                // MANUAL_PUMP_RUNTIME pulse instead of self-stopping the
                // instant it re-enters the (tighter) completion band - the
                // pulse duration is what makes this safe, not this check.
                if (!statuses[actuator].overrideActive)
                {
                    const float phUpLimit = runningValidation
                        ? systemState.phTargetMin : systemState.minPH;
                    const float phDownLimit = runningValidation
                        ? systemState.phTargetMax : systemState.maxPH;
                    if (actuator == PH_UP_PUMP && sensors.ph >= phUpLimit)
                    {
                        outReason = runningValidation
                            ? "pH Up complete: Minimum pH threshold reached."
                            : "pH Up rejected: Current pH does not require an increase.";
                        return false;
                    }

                    if (actuator == PH_DOWN_PUMP && sensors.ph <= phDownLimit)
                    {
                        outReason = runningValidation
                            ? "pH Down complete: Maximum pH threshold reached."
                            : "pH Down rejected: Current pH does not require a decrease.";
                        return false;
                    }
                }
            }

            if (manual && (actuator == GROW_PUMP || actuator == BLOOM_PUMP))
            {
                if (!validEC(sensors.ec))
                {
                    outReason = "Cannot dose: Current EC reading is invalid.";
                    return false;
                }

                // SOFT rule ("EC already inside the configured target range") -
                // same override treatment as the pH block above.
                if (!statuses[actuator].overrideActive)
                {
                    const float ecLimit = runningValidation
                        ? systemState.ecTargetMin : systemState.minEC;
                    if (sensors.ec >= ecLimit)
                    {
                        outReason = runningValidation
                            ? "Nutrient dosing complete: Minimum EC threshold reached."
                            : "Nutrient dosing rejected: Current EC does not require correction.";
                        return false;
                    }
                }
            }
            break;
        }
        case CANOPY_FAN:
            break;
        case SOLENOID:
        {
            if (systemState.refillSubsystemLocked)
            {
                outReason = "Refill/dilution subsystem is locked.";
                return false;
            }
            if (!validPercentage(sensors.waterLevel))
            {
                outReason = "Cannot refill: Water-level reading is invalid.";
                return false;
            }
            const bool dilutionOwnsLock =
                systemState.currentMode == DOSING_EC &&
                systemState.ecDirection == EC_DILUTE;
            if (systemState.reservoirLocked && systemState.currentMode != REFILLING && !dilutionOwnsLock)
            {
                outReason = "Cannot refill: Reservoir is locked by another operation.";
                return false;
            }
            if (isOn(PH_UP_PUMP) || isOn(PH_DOWN_PUMP) || isOn(GROW_PUMP) || isOn(BLOOM_PUMP))
            {
                outReason = "Cannot refill while dosing pumps are active.";
                return false;
            }
            if (manual)
            {
                if (!validPercentage(sensors.waterLevel))
                {
                    outReason = "Cannot refill: Current water-level reading is invalid.";
                    return false;
                }

                // SOFT rule ("refill not currently required") - overridable.
                if (!statuses[actuator].overrideActive)
                {
                    const float threshold = runningValidation
                        ? systemState.refillStopLevel
                        : systemState.refillStartLevel;
                    if (sensors.waterLevel >= threshold)
                    {
                        outReason = runningValidation
                            ? "Refill complete: Stop level reached."
                            : "Refill rejected: Water level is above the refill start threshold.";
                        return false;
                    }
                }
            }
            break;
        }
        case PELTIER:
            if (systemState.coolingSubsystemLocked)
            {
                if (manual) automationManager.setManualCoolingDemand(false);
                outReason = "Cooling subsystem is locked.";
                return false;
            }
            if (!validPercentage(sensors.waterLevel))
            {
                if (manual) automationManager.setManualCoolingDemand(false);
                outReason = "Peltier stopped. Water-level reading is invalid.";
                return false;
            }
            if (sensors.waterLevel < systemState.refillStartLevel)
            {
                if (manual) automationManager.setManualCoolingDemand(false);
                outReason = "Peltier stopped. Water reservoir level is too low.";
                return false;
            }
            if (!isfinite(sensors.waterTemp) || sensors.waterTemp < 0.0f || sensors.waterTemp > 100.0f)
            {
                if (manual) automationManager.setManualCoolingDemand(false);
                outReason = "Peltier stopped. Water-temperature reading is invalid.";
                return false;
            }
            if (manual)
            {
                // SOFT rule ("water temperature already within target") -
                // overridable. NOTE: unlike the dosing pumps, Peltier's
                // manual runtime cap is the much longer OPERATION_TIMEOUT_MS
                // (5 min, Config.h) - an override here can keep cooling
                // running for up to that whole window rather than a short
                // pulse. Flagged for a policy decision; not changed here.
                if (!statuses[actuator].overrideActive)
                {
                    const float threshold = runningValidation
                        ? systemState.coolerOffTemp
                        : systemState.highWaterTemp;
                    if (sensors.waterTemp <= threshold)
                    {
                        automationManager.setManualCoolingDemand(false);
                        outReason = runningValidation
                            ? "Cooling complete: Cooler-off temperature reached."
                            : "Peltier rejected: Water temperature does not require cooling.";
                        return false;
                    }
                }

                // Contribute to the existing PELTIER demand bit before waiting
                // for the automation-owned circulation actuator to acknowledge.
                automationManager.setManualCoolingDemand(true);
            }
            {
                const ActuatorStatus& circulation = statuses[CIRCULATION_PUMP];
                if (!isOn(CIRCULATION_PUMP) ||
                    !circulation.running ||
                    circulation.state != ActuatorCommandState::RUNNING)
                {
                    outReason = WAITING_FOR_CIRCULATION;
                    return false;
                }
            }
            break;
        case CIRCULATION_PUMP:
            if (!validPercentage(sensors.waterLevel))
            {
                outReason = "Circulation pump stopped. Water-level reading is invalid.";
                return false;
            }
            if (sensors.waterLevel < systemState.refillStartLevel)
            {
                outReason = "Circulation pump stopped. Water reservoir level is too low.";
                return false;
            }
            break;
        case FOGGER:
            if (statuses[actuator].source == "automatic")
            {
                SafetyResult fogSafety = safetyManager.canFog();
                if (fogSafety != SafetyResult::SAFE)
                {
                    outReason = safetyManager.getSafetyReason(fogSafety);
                    return false;
                }
            }
            if (!validPercentage(sensors.waterLevel))
            {
                outReason = "Fogger stopped. Water-level reading is invalid.";
                return false;
            }
            if (sensors.waterLevel < systemState.refillStartLevel)
            {
                outReason = "Fogger stopped. Water reservoir level is too low.";
                return false;
            }
            if (!validEnvironment())
            {
                outReason = "Fogger stopped. Environmental sensor reading is invalid.";
                return false;
            }
            break;
        case BLOWER:
            // Automatic blower demand is the delivery half of an automatic fog
            // request. Because FOGGER precedes BLOWER in the actuator update,
            // this accepts only after the paired fogger has actually started.
            // Manual blower control retains its existing independent behavior.
            if (statuses[actuator].source == "automatic")
            {
                const ActuatorStatus& fogger = statuses[FOGGER];
                if (!isOn(FOGGER) || !fogger.running ||
                    fogger.state != ActuatorCommandState::RUNNING)
                {
                    outReason = "Blower blocked: automatic fogger is not running.";
                    return false;
                }
            }
            break;
        default:
            break;
    }

    return true;
}

void ActuatorManager::update()
{
    unsigned long currentMillis = millis();

    if (!systemState.manualMode)
    {
        for (int i = 0; i < ACTUATOR_COUNT; i++)
        {
            manuallyOverridden[i] = false;
        }
    }

    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        Actuator a = (Actuator)i;
        ActuatorCommand& cmd = commands[i];
        ActuatorStatus& status = statuses[i];

        // Independent-deadline reconciliation. The esp_timer callback (a
        // different task context - see onDeadlineExpired()) has already
        // forced the physical GPIO OFF the instant the deadline elapsed,
        // regardless of whether loop() was stalled. This only catches up
        // ActuatorManager's own FSM/status bookkeeping to match that
        // physical reality, so actuatorStatus and Android cannot keep
        // showing RUNNING for an actuator that is already physically off.
        // Runs before the normal cmd/FSM processing below so it always takes
        // precedence for this tick.
        if (deadlineExpired[i])
        {
            deadlineExpired[i] = false;
            if (status.state == ActuatorCommandState::RUNNING ||
                status.state == ActuatorCommandState::ACTIVATING)
            {
                turnOff(a);
                status.running = false;
                status.state = ActuatorCommandState::OFF;
                status.reason = "Safety limit: independent deadline expired";
                manuallyOverridden[i] = false;
                statusDirty = true;
            }
        }

        const ActuatorStatus previousStatus = status;

        if (cmd.isPending)
        {
            cmd.isPending = false;
            status.source = cmd.source;
            status.speed = cmd.speed;
            status.strategy = cmd.source == "automatic" ? cmd.strategy : "";
            status.reason = cmd.reason;
            // Mirrors onto status (not just the transient command) so the
            // continuous RUNNING-state re-check below can see it for as long
            // as this actuator keeps running under this command.
            status.overrideActive = cmd.overrideRequested;

            if (cmd.targetState)
            {
                if (status.state == ActuatorCommandState::OFF || status.state == ActuatorCommandState::REJECTED || status.state == ActuatorCommandState::STOPPING)
                {
                    status.state = ActuatorCommandState::COMMAND_RECEIVED;
                }
            }
            else
            {
                if (status.state == ActuatorCommandState::RUNNING || status.state == ActuatorCommandState::ACTIVATING)
                {
                    status.state = ActuatorCommandState::STOPPING;
                    if (status.source == "manual" && status.reason.isEmpty())
                    {
                        status.reason = "Manual stop";
                    }
                }
                else
                {
                    status.state = ActuatorCommandState::OFF;
                }
            }
        }

        bool stateChanged = true;
        while (stateChanged)
        {
            stateChanged = false;
            switch (status.state)
            {
                case ActuatorCommandState::OFF:
                    break;

                case ActuatorCommandState::COMMAND_RECEIVED:
                    if (status.source == "manual")
                    {
                        Serial.print("[MANUAL] ");
                        Serial.print(actuatorLogName(a));
                        Serial.println(" validating");
                    }
                    status.state = ActuatorCommandState::VALIDATING;
                    stateChanged = true;
                    break;

                case ActuatorCommandState::VALIDATING:
                {
                    String reason;
                    if (validateCommand(a, true, reason))
                    {
                        if (status.source == "manual")
                        {
                            Serial.println("[MANUAL] accepted");
                        }
                        status.state = ActuatorCommandState::ACTIVATING;
                        stateChanged = true;
                    }
                    else
                    {
                        if (a == PELTIER && reason == WAITING_FOR_CIRCULATION)
                        {
                            // AutomationManager owns the circulation demand.
                            // Keep this request in VALIDATING until circulation
                            // has physically reached RUNNING.
                            status.reason = reason;
                            break;
                        }

                        if (status.source == "manual")
                        {
                            Serial.print("[MANUAL] REJECTED: ");
                            Serial.println(reason);
                            manuallyOverridden[i] = false;
                            if (a == PELTIER)
                            {
                                automationManager.setManualCoolingDemand(false);
                            }
                        }
                        status.state = ActuatorCommandState::REJECTED;
                        status.reason = reason;
                        stateChanged = true;
                    }
                    break;
                }

                case ActuatorCommandState::REJECTED:
                    break;

                case ActuatorCommandState::ACTIVATING:
                    turnOn(a);
                    status.running = true;
                    status.startedAt = currentMillis;
                    status.state = ActuatorCommandState::RUNNING;
                    stateChanged = true;
                    break;

                case ActuatorCommandState::RUNNING:
                {
                    // Check local safety timeout watchdogs
                    unsigned long runTime = currentMillis - status.startedAt;
                    bool timeoutExceeded = false;
                    
                    if (a == PH_UP_PUMP || a == PH_DOWN_PUMP || a == GROW_PUMP || a == BLOOM_PUMP)
                    {
                        const unsigned long maxRunTime = isManualSource(status.source)
                            ? MANUAL_PUMP_RUNTIME
                            : 60000UL;
                        if (runTime >= maxRunTime)
                        {
                            timeoutExceeded = true;
                            status.reason = "Safety limit: dosing pump running too long";
                        }
                    }
                    else if (a == SOLENOID)
                    {
                        const unsigned long maxRunTime = isManualSource(status.source)
                            ? OPERATION_TIMEOUT_MS
                            : 600000UL;
                        if (runTime >= maxRunTime)
                        {
                            timeoutExceeded = true;
                            status.reason = "Safety limit: solenoid running too long";
                        }
                    }
                    else if (isManualSource(status.source) && (a == PELTIER || a == FOGGER))
                    {
                        if (runTime >= OPERATION_TIMEOUT_MS)
                        {
                            timeoutExceeded = true;
                            status.reason = "Safety limit: manual actuator running too long";
                        }
                    }
                    
                    if (timeoutExceeded)
                    {
                        status.state = ActuatorCommandState::STOPPING;
                        stateChanged = true;
                        break;
                    }

                    String reason;
                    if (!validateCommand(a, true, reason, true))
                    {
                        status.state = ActuatorCommandState::STOPPING;
                        status.reason = reason;
                        stateChanged = true;
                    }
                    else
                    {
                        if (a == CANOPY_FAN)
                        {
                            uint8_t pwmValue = (uint8_t)((status.speed / 100.0f) * 255.0f);
                            ledcWrite(getPin(a), pwmValue);
                        }
                    }
                    break;
                }

                case ActuatorCommandState::STOPPING:
                    if (a == CIRCULATION_PUMP && isOn(PELTIER))
                    {
                        turnOff(PELTIER);
                        statuses[PELTIER].running = false;
                        statuses[PELTIER].state = ActuatorCommandState::OFF;
                        statuses[PELTIER].reason = "Peltier stopped: Circulation unavailable.";
                        manuallyOverridden[PELTIER] = false;
                        // Out-of-band stop (this is CIRCULATION_PUMP's own
                        // iteration, not PELTIER's) - the generic
                        // running-transition arm/disarm below only observes
                        // transitions within an actuator's own iteration, so
                        // PELTIER's independent deadline needs this explicit
                        // disarm to avoid firing into whatever PELTIER runs
                        // next.
                        disarmDeadline(PELTIER);
                        automationManager.setManualCoolingDemand(false);
                        systemState.coolingSubsystemLocked = true;
                        statusDirty = true;
                        Serial.println("[SAFETY] PELTIER stopped: circulation unavailable");
                    }
                    turnOff(a);
                    status.running = false;
                    if (isManualSource(status.source))
                    {
                        manuallyOverridden[i] = false;
                        if (a == PELTIER)
                        {
                            automationManager.setManualCoolingDemand(false);
                        }
                    }
                    status.state = ActuatorCommandState::OFF;
                    stateChanged = true;
                    break;
            }
        }

        // Single point of truth for arming/disarming this actuator's
        // independent deadline, covering every path that can start or stop
        // it this tick (explicit command, hard-safety stop, soft-rule stop,
        // loop-polled timeout) without needing a call at each individual
        // site. The one exception - PELTIER forced off from CIRCULATION_PUMP's
        // own iteration above - is out-of-band and disarms itself explicitly
        // where that happens, since it is not observable as a transition
        // within PELTIER's own iteration.
        if (!previousStatus.running && status.running)
        {
            if (isDeadlineProtected(a) && isManualSource(status.source))
            {
                armDeadline(a, manualDeadlineMs(a));
            }
        }
        else if (previousStatus.running && !status.running)
        {
            disarmDeadline(a);
        }

        if (actuatorStatusDiffers(previousStatus, status))
        {
            statusDirty = true;

            if (previousStatus.state != status.state ||
                previousStatus.running != status.running ||
                previousStatus.source != status.source)
            {
                Serial.print("[ACTUATOR] ");
                Serial.print(actuatorLogName(a));
                Serial.print(" ");
                Serial.println(actuatorStateLogName(status.state));
            }

            // History-correctness hook: FOGGER's `running` flip is the
            // CONFIRMED transition (ACTIVATING->RUNNING / STOPPING->OFF
            // above), never the request in COMMAND_RECEIVED/VALIDATING - so
            // this fires once per actual physical transition, matching what
            // Fogging Reports must reconstruct sessions from.
            if (a == FOGGER && previousStatus.running != status.running)
            {
                foggingEventQueue.recordConfirmedTransition(
                    status.running, status.source, status.strategy, status.reason);
            }
        }
    }
}
