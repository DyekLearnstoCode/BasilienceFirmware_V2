#include "SafetyManager.h"

#include "Globals.h"

namespace
{
    bool validPH()
    {
        return isfinite(sensors.ph) && sensors.ph >= 0.0f && sensors.ph <= 14.0f;
    }

    bool validEC()
    {
        return isfinite(sensors.ec) && sensors.ec >= 0.0f;
    }

    bool validWaterLevel()
    {
        return isfinite(sensors.waterLevel) &&
            sensors.waterLevel >= 0.0f && sensors.waterLevel <= 100.0f;
    }

    bool validWaterTemperature()
    {
        return isfinite(sensors.waterTemp) &&
            sensors.waterTemp >= 0.0f && sensors.waterTemp <= 100.0f;
    }

    bool validEnvironment()
    {
        return isfinite(sensors.temperature) &&
            sensors.temperature >= -40.0f && sensors.temperature <= 100.0f &&
            isfinite(sensors.humidity) &&
            sensors.humidity >= 0.0f && sensors.humidity <= 100.0f;
    }
}

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

    const bool phCorrectionOwnsLock =
        systemState.currentMode == DOSING_PH ||
        systemState.currentMode == STABILIZING_PH;

    if(systemState.reservoirLocked && !phCorrectionOwnsLock)
    {
        return SafetyResult::RESERVOIR_LOCK;
    }

    if(!validWaterLevel() || !validPH())
    {
        return SafetyResult::SENSOR_FAULT;
    }

    if(sensors.waterLevel < systemState.refillStartLevel)
    {
        return SafetyResult::LOW_WATER;
    }

    return SafetyResult::SAFE;
}

SafetyResult SafetyManager::canDoseEC() const
{
    if(systemState.safetyLock)
    {
        return SafetyResult::SAFETY_LOCKED;
    }

    const bool ecCorrectionOwnsLock =
        systemState.currentMode == DOSING_EC ||
        systemState.currentMode == STABILIZING_EC;

    if(systemState.reservoirLocked && !ecCorrectionOwnsLock)
    {
        return SafetyResult::RESERVOIR_LOCK;
    }

    if(!validWaterLevel() || !validEC())
    {
        return SafetyResult::SENSOR_FAULT;
    }

    if(sensors.waterLevel < systemState.refillStartLevel)
    {
        return SafetyResult::LOW_WATER;
    }

    return SafetyResult::SAFE;
}

SafetyResult SafetyManager::canRefill() const
{
    if(systemState.safetyLock)
    {
        return SafetyResult::SAFETY_LOCKED;
    }

    if(!validWaterLevel())
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

    if(!validWaterLevel() || !validEnvironment())
    {
        return SafetyResult::SENSOR_FAULT;
    }

    if(sensors.waterLevel < systemState.refillStartLevel)
    {
        return SafetyResult::LOW_WATER;
    }

    return SafetyResult::SAFE;
}

SafetyResult SafetyManager::canCool() const
{
    if(systemState.safetyLock)
    {
        return SafetyResult::SAFETY_LOCKED;
    }

    if(!validWaterLevel() || !validWaterTemperature())
    {
        return SafetyResult::SENSOR_FAULT;
    }

    if(sensors.waterLevel < systemState.refillStartLevel)
    {
        return SafetyResult::LOW_WATER;
    }

    return SafetyResult::SAFE;
}
SafetyResult SafetyManager::canResetSafety() const
{
    if(alertState.sensorFault)
    {
        return SafetyResult::SENSOR_FAULT;
    }
    return SafetyResult::SAFE;
}
