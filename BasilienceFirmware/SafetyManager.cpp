#include "SafetyManager.h"

#include "Globals.h"

void SafetyManager::begin()
{
}

void SafetyManager::update()
{
}

SafetyResult SafetyManager::canDosePH() const
{
    if(systemState.safetyLock)
    {
        return SafetyResult::SAFETY_LOCK;
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
        return SafetyResult::SAFETY_LOCK;
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
        return SafetyResult::SAFETY_LOCK;
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
        return SafetyResult::SAFETY_LOCK;
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
        return SafetyResult::SAFETY_LOCK;
    }

    if(alertState.sensorFault)
    {
        return SafetyResult::SENSOR_FAULT;
    }

    return SafetyResult::SAFE;
}