#include "ActuatorManager.h"

#include <Arduino.h>

#include "Config.h"

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

            WATER_HEATER_PIN,
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
        pinMode(getPin((Actuator)i), OUTPUT);

        digitalWrite(getPin((Actuator)i), LOW);

        actuatorStates[i] = false;
    }
}

void ActuatorManager::turnOn(Actuator actuator)
{
    digitalWrite(getPin(actuator), HIGH);

    actuatorStates[actuator] = true;
}

void ActuatorManager::turnOff(Actuator actuator)
{
    digitalWrite(getPin(actuator), LOW);

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
        turnOff((Actuator)i);
    }
}