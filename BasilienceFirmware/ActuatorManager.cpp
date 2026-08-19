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
            left.reason != right.reason;
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
        // TEMPORARY: while the raw GSM UART diagnostic is active, GPIO16/17
        // (CANOPY_FAN_PIN/PELTIER_PIN) are reused as GsmRawUartTest's RX/TX
        // pins - see Config.h. Neither pin may be configured or written to
        // by this subsystem during that window, so it never contends with
        // GsmRawUartTest for the same physical lines. Logical bookkeeping
        // below still initializes normally either way.
        const bool gsmOwnsThisPin =
            GSM_RAW_UART_TEST && (i == CANOPY_FAN || i == PELTIER);

        if (!gsmOwnsThisPin)
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
        }

        actuatorStates[i] = false;

        commands[i].isPending = false;
        statuses[i].state = ActuatorCommandState::OFF;
        statuses[i].running = false;
        manuallyOverridden[i] = false;
    }
}

void ActuatorManager::turnOn(Actuator actuator)
{
    // See begin() - GPIO16/17 belong exclusively to GsmRawUartTest while
    // the raw UART diagnostic is active.
    const bool gsmOwnsThisPin =
        GSM_RAW_UART_TEST && (actuator == CANOPY_FAN || actuator == PELTIER);

    if (actuator != CANOPY_FAN && !gsmOwnsThisPin)
    {
        digitalWrite(getPin(actuator), HIGH);
    }
    // For CANOPY_FAN, speed is applied dynamically in update()

    actuatorStates[actuator] = true;
}

void ActuatorManager::turnOff(Actuator actuator)
{
    // See begin() - GPIO16/17 belong exclusively to GsmRawUartTest while
    // the raw UART diagnostic is active.
    const bool gsmOwnsThisPin =
        GSM_RAW_UART_TEST && (actuator == CANOPY_FAN || actuator == PELTIER);

    if (!gsmOwnsThisPin)
    {
        if (actuator == CANOPY_FAN)
        {
            ledcWrite(getPin(actuator), 0);
        }
        else
        {
            digitalWrite(getPin(actuator), LOW);
        }
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

void ActuatorManager::requestCommand(Actuator actuator, bool state, const String& source, double timestamp, uint8_t speed, const String& strategy, const String& reason)
{
    if (actuator == CIRCULATION_PUMP && source != "automatic")
    {
        Serial.println("[CIRCULATION] Manual command ignored; pump is automation-managed");
        return;
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

            if (manual && (actuator == GROW_PUMP || actuator == BLOOM_PUMP))
            {
                if (!validEC(sensors.ec))
                {
                    outReason = "Cannot dose: Current EC reading is invalid.";
                    return false;
                }

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
        const ActuatorStatus previousStatus = status;

        if (cmd.isPending)
        {
            cmd.isPending = false;
            status.source = cmd.source;
            status.speed = cmd.speed;
            status.strategy = cmd.source == "automatic" ? cmd.strategy : "";
            status.reason = cmd.reason;
            
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
                        // See begin() - GPIO17 belongs exclusively to
                        // GsmRawUartTest while the raw UART diagnostic is
                        // active; no PWM write reaches it in that window.
                        if (a == CANOPY_FAN && !GSM_RAW_UART_TEST)
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
        }
    }
}
