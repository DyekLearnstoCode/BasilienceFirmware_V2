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
        static uint8_t invalidStreak = 0;
        const bool rawValid = isfinite(sensors.waterLevel) &&
            sensors.waterLevel >= 0.0f && sensors.waterLevel <= 100.0f;
        return debouncedValid(rawValid, invalidStreak);
    }

    bool validWaterTemperature()
    {
        static uint8_t invalidStreak = 0;
        const bool rawValid = isfinite(sensors.waterTemp) &&
            sensors.waterTemp >= 0.0f && sensors.waterTemp <= 100.0f;
        return debouncedValid(rawValid, invalidStreak);
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

    // Developer water-level override (Types.h's ignoreWaterLevelAutomation):
    // waives ONLY this reservoir-too-low reason for physical testing with a
    // reservoir intentionally kept below refillStartLevelCm. Every other
    // check in this function - locks, sensor validity - stays enforced
    // exactly as before. See ActuatorManager::lowWaterBlocks() for the
    // matching actuator-level gate and its bypass log.
    //
    // refillStartLevelCm (2.0cm), NOT criticalLowWaterCm (1.0cm) - see the
    // static automation integration audit. criticalLowWaterCm is a stricter,
    // separate escalation threshold ON TOP of this operational bar, not a
    // replacement for it; the operational "block dependent controllers"
    // boundary has always been the same 2.0cm level that makes refill
    // eligible, matching the pre-water-depth-model design where a single
    // shared threshold played both roles.
    if(!systemState.ignoreWaterLevelAutomation &&
       sensors.waterLevelCm <= systemState.refillStartLevelCm)
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

    // refillStartLevelCm, not criticalLowWaterCm - see canDosePH()'s matching
    // comment.
    if(!systemState.ignoreWaterLevelAutomation &&
       sensors.waterLevelCm <= systemState.refillStartLevelCm)
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
    if(sensors.waterLevelCm >= systemState.refillStopLevelCm &&
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

    // DHT/environment validity is deliberately NOT checked here - see the
    // automation resilience pass report. Root fogging's hard requirements
    // are water/pH/EC/ownership only; DHT selects the fog cadence
    // (AutomationManager::processFogCycle()) but never gates permission to
    // fog. This is an intentional behavior change from the prior design.
    if(!validWaterLevel())
    {
        return SafetyResult::SENSOR_FAULT;
    }

    if(!validPH())
    {
        return SafetyResult::SENSOR_FAULT;
    }

    // sensors.ph is the stable-value filter's authoritative output (see
    // SensorManager::applyEffectiveSensors()), so this plain comparison
    // no longer needs its own decision-layer hysteresis.
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

    // refillStartLevelCm, not criticalLowWaterCm - see canDosePH()'s matching
    // comment.
    if(!systemState.ignoreWaterLevelAutomation &&
       sensors.waterLevelCm <= systemState.refillStartLevelCm)
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

    // refillStartLevelCm, not criticalLowWaterCm - see canDosePH()'s matching
    // comment.
    if(!systemState.ignoreWaterLevelAutomation &&
       sensors.waterLevelCm <= systemState.refillStartLevelCm)
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

    // Requiring the reading to already be back in range here made the lock a
    // deadlock: the subsystem trips after MAX_PH_ATTEMPTS/MAX_EC_ATTEMPTS
    // failed automatic corrections specifically because automation could not
    // get the reading into range, so a reset that demanded that same
    // condition could never actually succeed while the real problem
    // persisted - only a human manually dosing the reservoir by hand could
    // clear it. Reset Safety is the admin's explicit request for a fresh
    // attempt, not a claim that the problem is already fixed; it still
    // requires the sensor itself to be reporting a real, physically valid
    // number (validPH()/validEC()) so a reset is never granted against a
    // broken/disconnected probe, but no longer requires the chemistry to
    // already be corrected. processResetSafetyOperation() zeroes
    // phAttempts/ecAttempts right after this succeeds, so automation gets a
    // genuine fresh MAX_*_ATTEMPTS run rather than restarting mid-count.
    if(systemState.phSubsystemLocked)
    {
        if(validPH())
        {
            systemState.phSubsystemLocked = false;
            cleared = true;
        }
        else reason += "pH subsystem remains unsafe. ";
    }

    if(systemState.ecSubsystemLocked)
    {
        if(validEC())
        {
            systemState.ecSubsystemLocked = false;
            cleared = true;
            Serial.println("[EC-LOCK] cleared reason=reset_safety");
        }
        else reason += "EC subsystem remains unsafe. ";
    }

    if(systemState.refillSubsystemLocked)
    {
        if(validWaterLevel())
        {
            systemState.refillSubsystemLocked = false;
            cleared = true;

            // Temporary diagnostic: prove exactly which admin Reset Safety
            // request cleared the lock, and when - see the root-cause
            // finding above the requestId gate in
            // AutomationManager::createOperationRequest().
            Serial.print("[REFILL-LOCK] CLEAR-WRITER source=reset_safety requestId=");
            Serial.print(systemState.operationRequest.requestId);
            Serial.print(" t=");
            Serial.print(millis());
            Serial.print(" water=");
            Serial.println(sensors.waterLevel, 2);
        }
        else reason += "Refill subsystem water-level input remains unsafe. ";
    }

    if(systemState.coolingSubsystemLocked)
    {
        // Mirrors canCool()'s own gate (refillStartLevelCm, not
        // criticalLowWaterCm) - this must clear exactly when canCool() would
        // newly report SAFE, or the lock could be released while canCool()
        // still blocks, or stay stuck after canCool() would already permit
        // cooling again.
        if(validWaterLevel() && validWaterTemperature() &&
           sensors.waterLevelCm > systemState.refillStartLevelCm)
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
