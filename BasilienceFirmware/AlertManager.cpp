#include "AlertManager.h"

#include "Globals.h"

namespace
{
    // Serial Monitor Focus Mode (tiny logging-only addition - see
    // DebugManager::shouldPrintDebug()'s own comment). Decides only whether
    // this alert's [ALERT] transition line prints; the alert STATE itself
    // (currentValue, alertsDirty - synced to Firebase/notifications exactly
    // as before) is set unconditionally by the caller regardless of this
    // return value. Every name not explicitly mapped here (target-range
    // temperature/humidity alerts, sensorFault's own fallback) is
    // conservatively suppressed while ANY controller is isolated - never
    // shown to the wrong one, only ever hidden from an unrelated one.
    bool alertRelevantToActiveTest(const char* name)
    {
        if (systemState.automationTestSubsystem == AutomationTestSubsystem::NONE)
            return true;

        if (strcmp(name, "lowWater") == 0 || strcmp(name, "criticalLowWater") == 0 ||
            strcmp(name, "waterLevelLow") == 0 || strcmp(name, "waterLevelHigh") == 0)
        {
            return debugManager.shouldPrintDebug(DebugCategory::WATER);
        }

        if (strcmp(name, "phLow") == 0 || strcmp(name, "phHigh") == 0 ||
            strcmp(name, "phOutOfRange") == 0)
        {
            return debugManager.shouldPrintDebug(DebugCategory::PH);
        }

        if (strcmp(name, "ecLow") == 0 || strcmp(name, "ecHigh") == 0)
        {
            return debugManager.shouldPrintDebug(DebugCategory::EC);
        }

        if (strcmp(name, "waterTempOutOfRange") == 0 || strcmp(name, "waterTempLow") == 0)
        {
            return debugManager.shouldPrintDebug(DebugCategory::COOLING);
        }

        // sensorFault spans waterTemp/pH/EC/waterLevel (see
        // AlertManager::updateSensorFaultAlert()) - relevant to any of the
        // reservoir-adjacent controllers, not suppressed for them.
        if (strcmp(name, "sensorFault") == 0)
        {
            return debugManager.shouldPrintDebug(DebugCategory::WATER) ||
                   debugManager.shouldPrintDebug(DebugCategory::PH) ||
                   debugManager.shouldPrintDebug(DebugCategory::EC) ||
                   debugManager.shouldPrintDebug(DebugCategory::COOLING);
        }

        return false;
    }
}

void AlertManager::begin()
{
    alertsDirty = true;
}

bool AlertManager::isDirty() const
{
    return alertsDirty;
}

void AlertManager::markSynced()
{
    alertsDirty = false;
}

void AlertManager::setAlert(const char* name, bool& currentValue, bool nextValue)
{
    if (currentValue == nextValue)
    {
        return;
    }

    currentValue = nextValue;
    alertsDirty = true;

    if (!alertRelevantToActiveTest(name)) return;

    Serial.print("[ALERT] ");
    Serial.print(name);
    Serial.print("=");
    Serial.print(nextValue ? "true" : "false");
    Serial.print(" t=");
    Serial.println(millis());
}

void AlertManager::setAlertDebounced(const char* name, bool& currentValue,
                                     bool nextValue, uint8_t& pendingCount)
{
    if (!nextValue)
    {
        pendingCount = 0;
        setAlert(name, currentValue, false);
        return;
    }

    if (pendingCount < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
    {
        pendingCount++;
    }

    setAlert(name, currentValue, pendingCount >= SENSOR_TRANSIENT_FAILURE_THRESHOLD);
}

void AlertManager::update()
{
    // Mirrors the same gate SensorManager applies to the effective sensor
    // dataset: while the mock-vs-physical source is still unresolved after
    // boot, sensors are held invalid, and no alert (including sensorFault)
    // should be derived from that transient window either. A boot-restored
    // mock session waiting for its first fresh payload is the same
    // situation - SensorManager::applyEffectiveSensors() is deliberately
    // publishing an all-NaN placeholder so automatic actuators fail closed
    // (unchanged, not touched here), and that placeholder must not also be
    // misread as a genuine sensor fault. Both checks only ever return early
    // - actuator/safety gating (SafetyManager's isfinite() checks against
    // the still-NaN `sensors`) is untouched by this file.
    if (!systemState.sensorSourceResolved || sensorManager.isMockBootWaiting())
    {
        return;
    }

    updateLowWaterAlert();

    updateTemperatureAlert();

    updateHumidityAlert();

    updateWaterTemperatureAlert();

    updatePHAlert();

    updateECAlert();

    updateSensorFaultAlert();
} 

void AlertManager::updateLowWaterAlert()
{
    const bool valid = isfinite(sensors.waterLevel);

    // CONTROL signal - stays on refillStartLevelCm (water depth, cm)
    // because it gates AutomationManager::handleNormal()'s automatic refill
    // trigger and SafetyManager::canResetSafety() - see Config.h's "Water
    // Reservoir Geometry" section. Deliberately NOT criticalLowWaterCm
    // (the stricter bar that blocks pH/EC/fogging/cooling directly in
    // SafetyManager/ActuatorManager, independent of this alert): this flag
    // means "eligible to refill," a materially less severe condition.
    // Retargeting this at minWaterLevel would change when the valve opens,
    // which is a control change, not a reporting one.
    setAlertDebounced("lowWater", alertState.lowWater,
        valid && sensors.waterLevelCm <= systemState.refillStartLevelCm,
        lowWaterPendingCount);

    // Severity escalation on top of lowWater - same debounce shape (see
    // setAlertDebounced()'s own comment), same `valid` NaN gate, so an
    // invalid/unavailable HC-SR04 reading can never raise this any more than
    // it can raise lowWater. No actuator gate reads this directly - the
    // <=2.0cm operational block above already covers pH/EC/fogging/cooling;
    // this is purely a status/notification severity signal for a reservoir
    // that has fallen even further, past criticalLowWaterCm.
    setAlertDebounced("criticalLowWater", alertState.criticalLowWater,
        valid && sensors.waterLevelCm <= systemState.criticalLowWaterCm,
        criticalLowWaterPendingCount);

    // TARGET-RANGE classification, reported alongside it.
    setAlertDebounced("waterLevelLow", alertState.waterLevelLow,
        valid && sensors.waterLevel < systemState.minWaterLevel,
        waterLevelLowPendingCount);

    setAlertDebounced("waterLevelHigh", alertState.waterLevelHigh,
        valid && sensors.waterLevel > systemState.maxWaterLevel,
        waterLevelHighPendingCount);
}

void AlertManager::updateTemperatureAlert()
{
    // dhtAvailable, not isfinite(sensors.temperature) - see the automation
    // resilience pass report. A held last-good reading (dhtStale=true) is
    // finite but must not drive a fresh target-range classification; only a
    // currently-fresh measurement should be able to raise/clear these.
    const bool valid = sensors.dhtAvailable;

    // Both sides now come from the configured target range. Previously the low
    // side compared against the hard-coded COLD_FOG_TEMPERATURE constant, which
    // was a fogging-strategy value rather than a user-facing bound. Canopy fan
    // control is unaffected: handleCanopyClimate() reads highAirTemp /
    // airTempRelease directly and never consults these flags.
    setAlertDebounced(
        "lowAirTemperature",
        alertState.lowAirTemperature,
        valid && sensors.temperature < systemState.minAirTemp,
        lowAirTemperaturePendingCount);

    setAlertDebounced(
        "highTemperature",
        alertState.highTemperature,
        valid && sensors.temperature > systemState.maxAirTemp,
        highTemperaturePendingCount);
}

void AlertManager::updateHumidityAlert()
{
    // dhtAvailable, not isfinite(sensors.humidity) - see
    // updateTemperatureAlert()'s matching comment.
    const bool valid = sensors.dhtAvailable;

    setAlertDebounced("humidityLow", alertState.humidityLow,
        valid && sensors.humidity < systemState.minHumidity,
        humidityLowPendingCount);

    setAlertDebounced("humidityHigh", alertState.humidityHigh,
        valid && sensors.humidity > systemState.maxHumidity,
        humidityHighPendingCount);
}

void AlertManager::updateWaterTemperatureAlert()
{
    // A missing reading is not an out-of-range reading: the old form compared
    // NaN directly, which silently evaluated false and reported "in range" for
    // a dead sensor. Peltier control is unaffected - updateCooling() reads
    // highWaterTemp / coolerOffTemp directly.
    const bool valid = isfinite(sensors.waterTemp);

    setAlertDebounced(
        "waterTempOutOfRange",
        alertState.waterTempOutOfRange,
        valid && sensors.waterTemp > systemState.maxWaterTemp,
        waterTempOutOfRangePendingCount);

    setAlertDebounced(
        "waterTempLow",
        alertState.waterTempLow,
        valid && sensors.waterTemp < systemState.minWaterTemp,
        waterTempLowPendingCount);
}



void AlertManager::updatePHAlert()
{
    // sensors.ph is now the stable-value filter's authoritative output (see
    // SensorManager::applyEffectiveSensors()/updateStabilityWindow()) - it
    // only ever changes when a new 10-sample window has actually agreed
    // within tolerance, so a plain single-threshold comparison here no
    // longer chatters. The decision-layer Schmitt-trigger hysteresis this
    // function used to carry (PH_ALERT_HYSTERESIS) was a second, overlapping
    // anti-flicker system on top of that upstream fix and has been removed;
    // minPH/maxPH stay the sole, simple, authoritative comparison.
    const bool valid = isfinite(sensors.ph) && sensors.ph >= 0.0f && sensors.ph <= 14.0f;
    const bool low = valid && sensors.ph < systemState.minPH;
    const bool high = valid && sensors.ph > systemState.maxPH;

    setAlertDebounced("phLow", alertState.phLow, low, phLowPendingCount);
    setAlertDebounced("phHigh", alertState.phHigh, high, phHighPendingCount);
    // Derived from the two flags above, which are already debounced - no
    // separate pending counter needed here.
    setAlert("phOutOfRange", alertState.phOutOfRange,
        alertState.phLow || alertState.phHigh);
}

void AlertManager::updateECAlert()
{
    setAlertDebounced(
        "ecLow",
        alertState.ecLow,
        isfinite(sensors.ec) && sensors.ec < systemState.minEC,
        ecLowPendingCount);

    setAlertDebounced(
        "ecHigh",
        alertState.ecHigh,
        isfinite(sensors.ec) && sensors.ec > systemState.maxEC,
        ecHighPendingCount);
}

void AlertManager::updateSensorFaultAlert()
{
    // RESERVOIR/ROOT-ZONE sensor fault only - see the automation resilience
    // pass report. DHT22 is an ENVIRONMENT/CANOPY sensor and is deliberately
    // NOT part of this aggregation any more: this flag feeds
    // SafetyManager::canResetSafety(), which gates whether the GLOBAL
    // systemState.safetyLock can ever clear, so an unstable DHT (a known,
    // recurring electrical-environment issue - see readDHT()) must never be
    // able to keep REFILL/PH/EC/COOLING locked out system-wide. DHT health is
    // published separately as sensors.dhtAvailable/dhtStale (see
    // FirebaseManager::writeSensors()) rather than folded back in here.
    const bool sensorFault =

        !isfinite(sensors.waterTemp) ||

        !isfinite(sensors.ph) ||

        !isfinite(sensors.ec) ||

        !isfinite(sensors.waterLevel) ||

        sensors.waterTemp < 0.0f ||
        sensors.waterTemp > 100.0f ||

        sensors.ph < 0.0f ||
        sensors.ph > 14.0f ||

        sensors.ec < 0.0f;

        // waterLevel is a percentage; zero is valid and means empty.
        // Values outside this range indicate an invalid effective reading.
    const bool waterLevelFault =
        isfinite(sensors.waterLevel) &&
        (sensors.waterLevel < 0.0f || sensors.waterLevel > 100.0f);

    const bool rawFault = sensorFault || waterLevelFault;

    // A single transient invalid tick must not immediately raise sensorFault.
    // Any valid tick resets the pending count right away so a real recovery
    // is never delayed; only SENSOR_TRANSIENT_FAILURE_THRESHOLD consecutive
    // invalid ticks actually raise it.
    if (rawFault)
    {
        if (sensorFaultPendingCount < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
        {
            sensorFaultPendingCount++;
        }
    }
    else
    {
        sensorFaultPendingCount = 0;
    }

    setAlert("sensorFault", alertState.sensorFault,
        sensorFaultPendingCount >= SENSOR_TRANSIENT_FAILURE_THRESHOLD);
}
