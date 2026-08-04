#include "ActuatorManager.h"

#include <Arduino.h>

#include "Config.h"
#include "Globals.h"

namespace
{
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
            PELTIER_PIN};
}

uint8_t ActuatorManager::getPin(Actuator actuator) const
{
    return actuatorPins[actuator];
}

void ActuatorManager::begin()
{
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

void ActuatorManager::turnOffAll()
{
    for (int i = 0; i < ACTUATOR_COUNT; i++)
    {
        requestCommand((Actuator)i, false, "automatic", millis());
    }
}

void ActuatorManager::requestCommand(Actuator actuator, bool state, const String& source, double timestamp, uint8_t speed)
{
    // Ignore automatic schedule commands when this actuator is manually overridden in manual mode
    if (systemState.manualMode && source == "automatic" && manuallyOverridden[actuator])
    {
        return;
    }

    if (source == "manual")
    {
        manuallyOverridden[actuator] = true;
    }

    commands[actuator].isPending = true;
    commands[actuator].targetState = state;
    commands[actuator].speed = speed;
    commands[actuator].source = source;
    commands[actuator].timestamp = timestamp;
}

ActuatorStatus ActuatorManager::getStatus(Actuator actuator) const
{
    return statuses[actuator];
}

bool ActuatorManager::validateCommand(Actuator actuator, bool targetState, String& outReason)
{
    if (!targetState) return true;

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
            if (systemState.reservoirLocked || isOn(SOLENOID))
            {
                outReason = "Reservoir is currently locked by another operation.";
                return false;
            }
            if (alertState.lowWater)
            {
                outReason = "Cannot dose: Water reservoir level is too low.";
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
            break;
        case CANOPY_FAN:
            break;
        case SOLENOID:
            if (isOn(PH_UP_PUMP) || isOn(PH_DOWN_PUMP) || isOn(GROW_PUMP) || isOn(BLOOM_PUMP))
            {
                outReason = "Cannot refill while dosing pumps are active.";
                return false;
            }
            break;
        case PELTIER:
            if (alertState.lowWater)
            {
                outReason = "Peltier stopped. Water reservoir level is too low.";
                return false;
            }
            break;
        case FOGGER:
            if (alertState.lowWater)
            {
                outReason = "Fogger stopped. Water reservoir level is too low.";
                return false;
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

        if (cmd.isPending)
        {
            cmd.isPending = false;
            status.source = cmd.source;
            status.speed = cmd.speed;
            
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
                    status.reason = "Manual stop";
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
                    status.state = ActuatorCommandState::VALIDATING;
                    stateChanged = true;
                    break;

                case ActuatorCommandState::VALIDATING:
                {
                    String reason;
                    if (validateCommand(a, true, reason))
                    {
                        status.state = ActuatorCommandState::ACTIVATING;
                        status.reason = "";
                        stateChanged = true;
                    }
                    else
                    {
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
                        if (runTime >= 60000UL) // 60 seconds
                        {
                            timeoutExceeded = true;
                            status.reason = "Safety limit: dosing pump running too long";
                        }
                    }
                    else if (a == SOLENOID)
                    {
                        if (runTime >= 600000UL) // 10 minutes
                        {
                            timeoutExceeded = true;
                            status.reason = "Safety limit: solenoid running too long";
                        }
                    }
                    
                    if (timeoutExceeded)
                    {
                        status.state = ActuatorCommandState::STOPPING;
                        stateChanged = true;
                        break;
                    }

                    String reason;
                    if (!validateCommand(a, true, reason))
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
                    turnOff(a);
                    status.running = false;
                    status.state = ActuatorCommandState::OFF;
                    stateChanged = true;
                    break;
            }
        }
    }
}