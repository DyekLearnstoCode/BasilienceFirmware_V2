#include "SafetyManager.h"

#include "Globals.h"

void SafetyManager::begin()
{
}

void SafetyManager::update()
{
}

const char* SafetyManager::getSafetyReason(
    SafetyResult result) const
{
    switch(result)
    {
        case SafetyResult::SAFE:
            return "Safe";

        case SafetyResult::LOW_WATER:
            return "Low water";

        case SafetyResult::SENSOR_FAULT:
            return "Sensor fault";

        case SafetyResult::HIGH_WATER_TEMP:
            return "High water temperature";

        case SafetyResult::SAFETY_LOCKED:
            return "Safety lock";

        case SafetyResult::RESERVOIR_LOCK:
            return "Reservoir locked";

        default:
            return "Unknown";
    }
}

SafetyResult SafetyManager::canDosePH() const
{
    if(systemState.safetyLock)
    {
        return SafetyResult::SAFETY_LOCKED;
    }

    if(systemState.reservoirLocked)
    {
        return SafetyResult::RESERVOIR_LOCK;
    }

    if(alertState.lowWater)
    {
        return SafetyResult::LOW_WATER;
    }

    if(alertState.sensorFault)
    {
        return SafetyResult::SENSOR_FAULT;
    }

    return SafetyResult::SAFE;
}

SafetyResult SafetyManager::canDoseEC() const
{
    if(systemState.safetyLock)
    {
        return SafetyResult::SAFETY_LOCKED;
    }

    if(systemState.reservoirLocked)
    {
        return SafetyResult::RESERVOIR_LOCK;
    }

    if(alertState.lowWater)
    {
        return SafetyResult::LOW_WATER;
    }

    if(alertState.sensorFault)
    {
        return SafetyResult::SENSOR_FAULT;
    }

    return SafetyResult::SAFE;
}

SafetyResult SafetyManager::canRefill() const
{
    if(systemState.safetyLock)
    {
        return SafetyResult::SAFETY_LOCKED;
    }

    if(alertState.sensorFault)
    {
        return SafetyResult::SENSOR_FAULT;
    }

    return SafetyResult::SAFE;
}

SafetyResult SafetyManager::canFog() const
{
    if(systemState.safetyLock)
    {
        return SafetyResult::SAFETY_LOCKED;
    }

    if(alertState.lowWater)
    {
        return SafetyResult::LOW_WATER;
    }

    if(alertState.sensorFault)
    {
        return SafetyResult::SENSOR_FAULT;
    }

    return SafetyResult::SAFE;
}

SafetyResult SafetyManager::canCool() const
{
    if(systemState.safetyLock)
    {
        return SafetyResult::SAFETY_LOCKED;
    }

    if(alertState.sensorFault)
    {
        return SafetyResult::SENSOR_FAULT;
    }

    return SafetyResult::SAFE;
}