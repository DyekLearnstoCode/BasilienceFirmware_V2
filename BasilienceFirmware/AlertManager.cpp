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
            strcmp(name, "waterLevelLow") == 0 || strcmp(name, "waterLevelHigh") == 0 ||
            strcmp(name, "refillIneffective") == 0)
        {
            return debugManager.shouldPrintDebug(DebugCategory::WATER);
        }

        if (strcmp(name, "phLow") == 0 || strcmp(name, "phHigh") == 0 ||
            strcmp(name, "phOutOfRange") == 0)
        {
            return debugManager.shouldPrintDebug(DebugCategory::PH);
        }

        if (strcmp(name, "ecLow") == 0 || strcmp(name, "ecHigh") == 0 ||
            strcmp(name, "ecDilutionIneffective") == 0)
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

void AlertManager::setAlertDebounced(const char* name, bool& currentValue, bool nextAbnormal,
                                     uint8_t& abnormalPendingCount, uint8_t& recoveryPendingCount,
                                     bool newSample)
{
    if (nextAbnormal)
    {
        recoveryPendingCount = 0;

        if (newSample && abnormalPendingCount < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
        {
            abnormalPendingCount++;
        }

        if (abnormalPendingCount >= SENSOR_TRANSIENT_FAILURE_THRESHOLD)
        {
            setAlert(name, currentValue, true);
        }
    }
    else
    {
        abnormalPendingCount = 0;

        if (newSample && recoveryPendingCount < SENSOR_TRANSIENT_FAILURE_THRESHOLD)
        {
            recoveryPendingCount++;
        }

        if (recoveryPendingCount >= SENSOR_TRANSIENT_FAILURE_THRESHOLD)
        {
            setAlert(name, currentValue, false);
        }
    }
}

bool AlertManager::newObservation(uint32_t currentVersion, uint32_t& lastProcessedVersion)
{
    if (currentVersion == lastProcessedVersion)
    {
        return false;
    }
    lastProcessedVersion = currentVersion;
    return true;
}

bool AlertManager::aboveWithHysteresis(bool currentlyActive, float value,
                                       float threshold, float hysteresis, bool valid)
{
    if (!valid) return false;
    return currentlyActive ? (value > threshold - hysteresis) : (value > threshold);
}

bool AlertManager::belowWithHysteresis(bool currentlyActive, float value,
                                       float threshold, float hysteresis, bool valid)
{
    if (!valid) return false;
    return currentlyActive ? (value < threshold + hysteresis) : (value < threshold);
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

    const bool dhtSampleReady = updateTemperatureAlert();

    updateHumidityAlert(dhtSampleReady);

    updateWaterTemperatureAlert();

    updatePHAlert();

    updateECAlert();

    updateSensorFaultAlert();
} 

void AlertManager::updateLowWaterAlert()
{
    const bool valid = isfinite(sensors.waterLevel);

    // Stage 2: one shared "genuinely new HC-SR04 observation" gate for all
    // four waterLevel/waterLevelCm-derived alerts below - see
    // newObservation()'s own comment. Sourced from SensorManager's own
    // waterLevelSampleVersion, which does NOT advance while the fogger has
    // skipped acquisition, a read isn't due yet, or a read failed/is still
    // reacquiring a baseline - only a genuinely accepted new depth.
    const bool newSample = newObservation(sensorManager.getWaterLevelSampleVersion(), waterLevelLastProcessedVersion);

    // CONTROL signal - stays on refillStartLevelCm (water depth, cm)
    // because it gates AutomationManager::handleNormal()'s automatic refill
    // trigger and SafetyManager::canResetSafety() - see Config.h's "Water
    // Reservoir Geometry" section. Deliberately NOT criticalLowWaterCm
    // (the stricter bar that blocks pH/EC/fogging/cooling directly in
    // SafetyManager/ActuatorManager, independent of this alert): this flag
    // means "eligible to refill," a materially less severe condition.
    // Retargeting this at minWaterLevel would change when the valve opens,
    // which is a control change, not a reporting one. A LOW-only condition
    // (no upper bound) - recovers once the depth has risen back past
    // refillStartLevelCm + WATER_LEVEL_CM_ALERT_HYSTERESIS, not merely back
    // over the same line the refill valve itself just opened at.
    setAlertDebounced("lowWater", alertState.lowWater,
        belowWithHysteresis(alertState.lowWater, sensors.waterLevelCm,
            systemState.refillStartLevelCm, WATER_LEVEL_CM_ALERT_HYSTERESIS, valid),
        lowWaterAbnormalPendingCount, lowWaterRecoveryPendingCount, newSample);

    // Severity escalation on top of lowWater - same debounce/hysteresis
    // shape, same `valid` NaN gate, so an invalid/unavailable HC-SR04
    // reading can never raise this any more than it can raise lowWater. No
    // actuator gate reads this directly - the <=2.0cm operational block
    // above already covers pH/EC/fogging/cooling; this is purely a
    // status/notification severity signal for a reservoir that has fallen
    // even further, past criticalLowWaterCm.
    setAlertDebounced("criticalLowWater", alertState.criticalLowWater,
        belowWithHysteresis(alertState.criticalLowWater, sensors.waterLevelCm,
            systemState.criticalLowWaterCm, WATER_LEVEL_CM_ALERT_HYSTERESIS, valid),
        criticalLowWaterAbnormalPendingCount, criticalLowWaterRecoveryPendingCount, newSample);

    // TARGET-RANGE classification, reported alongside it.
    setAlertDebounced("waterLevelLow", alertState.waterLevelLow,
        belowWithHysteresis(alertState.waterLevelLow, sensors.waterLevel,
            systemState.minWaterLevel, WATER_LEVEL_ALERT_HYSTERESIS, valid),
        waterLevelLowAbnormalPendingCount, waterLevelLowRecoveryPendingCount, newSample);

    setAlertDebounced("waterLevelHigh", alertState.waterLevelHigh,
        aboveWithHysteresis(alertState.waterLevelHigh, sensors.waterLevel,
            systemState.maxWaterLevel, WATER_LEVEL_ALERT_HYSTERESIS, valid),
        waterLevelHighAbnormalPendingCount, waterLevelHighRecoveryPendingCount, newSample);

    // AutomationManager::handleBoundedAutomaticRefill() already debounces
    // this itself (systemState.refillNoRiseStreak requires 2 consecutive
    // no-rise refill attempts before reaching 2) - no separate pendingCount
    // needed here, unlike the threshold alerts above. Mirrors
    // ecDilutionIneffective's exact same treatment in updateECAlert(). Not a
    // Stage 2 threshold alert - untouched.
    setAlert(
        "refillIneffective",
        alertState.refillIneffective,
        systemState.refillNoRiseStreak >= 2);
}

bool AlertManager::updateTemperatureAlert()
{
    // dhtAvailable, not isfinite(sensors.temperature) - see the automation
    // resilience pass report. A held last-good reading (dhtStale=true) is
    // finite but must not drive a fresh target-range classification; only a
    // currently-fresh measurement should be able to raise/clear these.
    const bool valid = sensors.dhtAvailable;

    // Stage 2: shared "genuinely new DHT observation" gate - one read
    // refreshes temperature AND humidity together (SensorManager's single
    // dhtSampleVersion), so updateHumidityAlert() reuses the same result
    // (passed back to update() below) rather than tracking its own. Does
    // NOT advance on a not-due-yet tick or a failed/held-stale read - only
    // a genuinely accepted new reading.
    const bool newSample = newObservation(sensorManager.getDhtSampleVersion(), dhtLastProcessedVersion);

    // Both sides now come from the configured target range. Previously the low
    // side compared against the hard-coded COLD_FOG_TEMPERATURE constant, which
    // was a fogging-strategy value rather than a user-facing bound. Canopy fan
    // control is unaffected: handleCanopyClimate() reads highAirTemp /
    // airTempRelease directly and never consults these flags.
    setAlertDebounced(
        "lowAirTemperature",
        alertState.lowAirTemperature,
        belowWithHysteresis(alertState.lowAirTemperature, sensors.temperature,
            systemState.minAirTemp, AIR_TEMP_ALERT_HYSTERESIS, valid),
        lowAirTemperatureAbnormalPendingCount, lowAirTemperatureRecoveryPendingCount, newSample);

    setAlertDebounced(
        "highTemperature",
        alertState.highTemperature,
        aboveWithHysteresis(alertState.highTemperature, sensors.temperature,
            systemState.maxAirTemp, AIR_TEMP_ALERT_HYSTERESIS, valid),
        highTemperatureAbnormalPendingCount, highTemperatureRecoveryPendingCount, newSample);

    return newSample;
}

void AlertManager::updateHumidityAlert(bool newSample)
{
    // dhtAvailable, not isfinite(sensors.humidity) - see
    // updateTemperatureAlert()'s matching comment. newSample is computed
    // once by updateTemperatureAlert() and passed in - same DHT read
    // refreshes both, so a second independent newObservation() call here
    // against the same dhtSampleVersion (already consumed into
    // dhtLastProcessedVersion by updateTemperatureAlert() this cycle) would
    // always read false.
    const bool valid = sensors.dhtAvailable;

    setAlertDebounced("humidityLow", alertState.humidityLow,
        belowWithHysteresis(alertState.humidityLow, sensors.humidity,
            systemState.minHumidity, HUMIDITY_ALERT_HYSTERESIS, valid),
        humidityLowAbnormalPendingCount, humidityLowRecoveryPendingCount, newSample);

    setAlertDebounced("humidityHigh", alertState.humidityHigh,
        aboveWithHysteresis(alertState.humidityHigh, sensors.humidity,
            systemState.maxHumidity, HUMIDITY_ALERT_HYSTERESIS, valid),
        humidityHighAbnormalPendingCount, humidityHighRecoveryPendingCount, newSample);
}

void AlertManager::updateWaterTemperatureAlert()
{
    // A missing reading is not an out-of-range reading: the old form compared
    // NaN directly, which silently evaluated false and reported "in range" for
    // a dead sensor. Peltier control is unaffected - updateCooling() reads
    // highWaterTemp / coolerOffTemp directly.
    const bool valid = isfinite(sensors.waterTemp);

    // Stage 2: shared "genuinely new DS18B20 observation" gate for both
    // directions below - does NOT advance on a not-due-yet tick or a
    // transient/confirmed failure.
    const bool newSample = newObservation(sensorManager.getWaterTempSampleVersion(), waterTempLastProcessedVersion);

    setAlertDebounced(
        "waterTempOutOfRange",
        alertState.waterTempOutOfRange,
        aboveWithHysteresis(alertState.waterTempOutOfRange, sensors.waterTemp,
            systemState.maxWaterTemp, WATER_TEMP_ALERT_HYSTERESIS, valid),
        waterTempOutOfRangeAbnormalPendingCount, waterTempOutOfRangeRecoveryPendingCount, newSample);

    // No active water-heating actuator exists in this design and none is
    // added by this alert - low water temperature is a MONITORED/ALERT
    // condition only, never an automatic control trigger. The ultrasonic
    // fogger's normal operation does produce PASSIVE warming of the
    // nutrient solution as a side effect of its own unrelated function
    // (misting the root chamber) - that is not, and must not be confused
    // with, a closed-loop heater: nothing here commands extra fogging (or
    // anything else) in response to this alert, and the programmed fog
    // cadence (AutomationManager::processFogCycle()) is untouched by water
    // temperature in either direction.
    setAlertDebounced(
        "waterTempLow",
        alertState.waterTempLow,
        belowWithHysteresis(alertState.waterTempLow, sensors.waterTemp,
            systemState.minWaterTemp, WATER_TEMP_ALERT_HYSTERESIS, valid),
        waterTempLowAbnormalPendingCount, waterTempLowRecoveryPendingCount, newSample);
}



void AlertManager::updatePHAlert()
{
    // sensors.ph is the live SensorManager-filtered value (Stage 1 of the
    // sensor architecture redesign - see SensorManager::applyEffectiveSensors())
    // and can now change every loop() tick, not just once per confirmed
    // stability-window agreement as before Stage 1 - so the raw threshold
    // compare alone would chatter again exactly the way the old, now-removed
    // PH_ALERT_HYSTERESIS was originally added to fix. Stage 2 reinstates
    // that anti-flicker margin at the alert layer specifically (Config.h's
    // PH_ALERT_HYSTERESIS), this time paired with genuine-observation-based
    // confirmation (newObservation()) instead of overlapping with an
    // upstream stability gate - see aboveWithHysteresis()/belowWithHysteresis().
    const bool valid = isfinite(sensors.ph) && sensors.ph >= 0.0f && sensors.ph <= 14.0f;
    const bool newSample = newObservation(sensorManager.getPhSampleVersion(), phLastProcessedVersion);
    const bool low = belowWithHysteresis(alertState.phLow, sensors.ph,
        systemState.minPH, PH_ALERT_HYSTERESIS, valid);
    const bool high = aboveWithHysteresis(alertState.phHigh, sensors.ph,
        systemState.maxPH, PH_ALERT_HYSTERESIS, valid);

    setAlertDebounced("phLow", alertState.phLow, low, phLowAbnormalPendingCount, phLowRecoveryPendingCount, newSample);
    setAlertDebounced("phHigh", alertState.phHigh, high, phHighAbnormalPendingCount, phHighRecoveryPendingCount, newSample);
    // Derived from the two flags above, which are already debounced - no
    // separate pending counter needed here.
    setAlert("phOutOfRange", alertState.phOutOfRange,
        alertState.phLow || alertState.phHigh);
}

void AlertManager::updateECAlert()
{
    const bool valid = isfinite(sensors.ec);
    const bool newSample = newObservation(sensorManager.getEcSampleVersion(), ecLastProcessedVersion);

    setAlertDebounced(
        "ecLow",
        alertState.ecLow,
        belowWithHysteresis(alertState.ecLow, sensors.ec, systemState.minEC, EC_ALERT_HYSTERESIS, valid),
        ecLowAbnormalPendingCount, ecLowRecoveryPendingCount, newSample);

    setAlertDebounced(
        "ecHigh",
        alertState.ecHigh,
        aboveWithHysteresis(alertState.ecHigh, sensors.ec, systemState.maxEC, EC_ALERT_HYSTERESIS, valid),
        ecHighAbnormalPendingCount, ecHighRecoveryPendingCount, newSample);

    // AutomationManager::handleStabilizingEC() already debounces this itself
    // (systemState.ecDilutionNoRiseStreak requires 2 consecutive no-rise
    // dilution intervals before reaching 2) - no separate pendingCount
    // needed here, unlike the threshold alerts above. Not a Stage 2
    // threshold alert - untouched.
    setAlert(
        "ecDilutionIneffective",
        alertState.ecDilutionIneffective,
        systemState.ecDilutionNoRiseStreak >= 2);
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
