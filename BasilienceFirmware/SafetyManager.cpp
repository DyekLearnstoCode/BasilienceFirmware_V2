#include "SafetyManager.h"

#include "Globals.h"

namespace
{
    // A single transient invalid tick (OneWire hiccup, ADC noise, a blocking
    // call landing at the wrong moment) must not abort an active automatic
    // operation. Each metric tracks its own short consecutive-invalid streak;
    // any valid reading clears it immediately, while a genuinely sustained
    // failure still reports invalid after the same short threshold used
    // elsewhere (sensorFault, water-temperature confirmation).
    bool debouncedValid(bool rawValid, uint8_t& invalidStreak)
    {
        if (rawValid)
        {
            invalidStreak = 0;
            return true;
        }

        if (invalidStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
        {
            invalidStreak++;
        }

        return invalidStreak < SENSOR_TRANSIENT_FAILURE_THRESHOLD;
    }

    bool validPH()
    {
        static uint8_t invalidStreak = 0;
        const bool rawValid =
            isfinite(sensors.ph) && sensors.ph >= 0.0f && sensors.ph <= 14.0f;
        return debouncedValid(rawValid, invalidStreak);
    }

    bool validEC()
    {
        static uint8_t invalidStreak = 0;
        const bool rawValid = isfinite(sensors.ec) && sensors.ec >= 0.0f;
        return debouncedValid(rawValid, invalidStreak);
    }

    bool validWaterLevel()
    {
        return isfinite(sensors.waterLevel) &&
            sensors.waterLevel >= 0.0f && sensors.waterLevel <= 100.0f;
    }

    bool validWaterTemperature()
    {
        static uint8_t invalidStreak = 0;
        const bool rawValid = isfinite(sensors.waterTemp) &&
            sensors.waterTemp >= 0.0f && sensors.waterTemp <= 100.0f;
        return debouncedValid(rawValid, invalidStreak);
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
    // Refresh local safety inputs even while the FSM is parked in SAFETY_LOCK.
    alertManager.update();
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

        case SafetyResult::INVALID_PH:
            return "pH remains outside the configured safe range";

        case SafetyResult::INVALID_EC:
            return "EC remains outside the configured safe range";

        case SafetyResult::SUBSYSTEM_LOCKED:
            return "Subsystem locked; reset required";

        case SafetyResult::RESERVOIR_FULL:
            return "Reservoir full; manual attention required";

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

    if(systemState.phSubsystemLocked)
    {
        return SafetyResult::SUBSYSTEM_LOCKED;
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

    if(systemState.ecSubsystemLocked)
    {
        return SafetyResult::SUBSYSTEM_LOCKED;
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

SafetyResult SafetyManager::canDiluteEC() const
{
    SafetyResult result = canDoseEC();
    if(result != SafetyResult::SAFE) return result;
    if(systemState.refillSubsystemLocked) return SafetyResult::SUBSYSTEM_LOCKED;
    if(sensors.waterLevel >= systemState.refillStopLevel &&
       sensors.ec > systemState.ecTargetMax) return SafetyResult::RESERVOIR_FULL;
    return SafetyResult::SAFE;
}

SafetyResult SafetyManager::canRefill() const
{
    if(systemState.safetyLock)
    {
        return SafetyResult::SAFETY_LOCKED;
    }

    if(systemState.refillSubsystemLocked)
    {
        return SafetyResult::SUBSYSTEM_LOCKED;
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

    if(!validPH())
    {
        return SafetyResult::SENSOR_FAULT;
    }

    if(sensors.ph < systemState.minPH || sensors.ph > systemState.maxPH)
    {
        return SafetyResult::INVALID_PH;
    }

    if(!validEC())
    {
        return SafetyResult::SENSOR_FAULT;
    }

    if(sensors.ec < systemState.minEC || sensors.ec > systemState.maxEC)
    {
        return SafetyResult::INVALID_EC;
    }

    if(systemState.currentMode == DOSING_PH || systemState.currentMode == STABILIZING_PH)
    {
        return SafetyResult::INVALID_PH;
    }

    if(systemState.currentMode == DOSING_EC || systemState.currentMode == STABILIZING_EC)
    {
        return SafetyResult::INVALID_EC;
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

    if(systemState.coolingSubsystemLocked)
    {
        return SafetyResult::SUBSYSTEM_LOCKED;
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
    if(alertState.lowWater)
    {
        return SafetyResult::LOW_WATER;
    }
    if(alertState.phOutOfRange)
    {
        return SafetyResult::INVALID_PH;
    }
    if(alertState.ecLow)
    {
        return SafetyResult::INVALID_EC;
    }
    return SafetyResult::SAFE;
}

bool SafetyManager::resetRecoverableSubsystems(String& reason)
{
    bool cleared = false;
    reason = "";

    if(systemState.phSubsystemLocked)
    {
        if(validPH() && sensors.ph >= systemState.minPH && sensors.ph <= systemState.maxPH)
        {
            systemState.phSubsystemLocked = false;
            cleared = true;
        }
        else reason += "pH subsystem remains unsafe. ";
    }

    if(systemState.ecSubsystemLocked)
    {
        if(validEC() && sensors.ec >= systemState.minEC && sensors.ec <= systemState.maxEC)
        {
            systemState.ecSubsystemLocked = false;
            cleared = true;
        }
        else reason += "EC subsystem remains unsafe. ";
    }

    if(systemState.refillSubsystemLocked)
    {
        if(validWaterLevel())
        {
            systemState.refillSubsystemLocked = false;
            cleared = true;
        }
        else reason += "Refill subsystem water-level input remains unsafe. ";
    }

    if(systemState.coolingSubsystemLocked)
    {
        if(validWaterLevel() && validWaterTemperature() &&
           sensors.waterLevel >= systemState.refillStartLevel)
        {
            systemState.coolingSubsystemLocked = false;
            cleared = true;
        }
        else reason += "Cooling subsystem remains unsafe. ";
    }

    if(systemState.safetyLock)
    {
        SafetyResult global = canResetSafety();
        if(global == SafetyResult::SAFE)
        {
            systemState.safetyLock = false;
            cleared = true;
        }
        else reason += String(getSafetyReason(global)) + ". ";
    }

    return cleared;
}
