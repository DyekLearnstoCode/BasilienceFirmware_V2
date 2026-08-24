#include "AlertManager.h"

#include "Globals.h"
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

    Serial.print("[ALERT] ");
    Serial.print(name);
    Serial.print("=");
    Serial.print(nextValue ? "true" : "false");
    Serial.print(" t=");
    Serial.println(millis());
}

void AlertManager::update()
{
    // Mirrors the same gate SensorManager applies to the effective sensor
    // dataset: while the mock-vs-physical source is still unresolved after
    // boot, sensors are held invalid, and no alert (including sensorFault)
    // should be derived from that transient window either.
    if (!systemState.sensorSourceResolved)
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

    // CONTROL signal - stays on refillStartLevel because automatic refill is
    // driven by it. Retargeting this at minWaterLevel would change when the
    // valve opens, which is a control change, not a reporting one.
    setAlert("lowWater", alertState.lowWater,
        valid && sensors.waterLevel < systemState.refillStartLevel);

    // TARGET-RANGE classification, reported alongside it.
    setAlert("waterLevelLow", alertState.waterLevelLow,
        valid && sensors.waterLevel < systemState.minWaterLevel);

    setAlert("waterLevelHigh", alertState.waterLevelHigh,
        valid && sensors.waterLevel > systemState.maxWaterLevel);
}

void AlertManager::updateTemperatureAlert()
{
    const bool valid = isfinite(sensors.temperature);

    // Both sides now come from the configured target range. Previously the low
    // side compared against the hard-coded COLD_FOG_TEMPERATURE constant, which
    // was a fogging-strategy value rather than a user-facing bound. Canopy fan
    // control is unaffected: handleCanopyClimate() reads highAirTemp /
    // airTempRelease directly and never consults these flags.
    setAlert(
        "lowAirTemperature",
        alertState.lowAirTemperature,
        valid && sensors.temperature < systemState.minAirTemp);

    setAlert(
        "highTemperature",
        alertState.highTemperature,
        valid && sensors.temperature > systemState.maxAirTemp);
}

void AlertManager::updateHumidityAlert()
{
    const bool valid = isfinite(sensors.humidity);

    setAlert("humidityLow", alertState.humidityLow,
        valid && sensors.humidity < systemState.minHumidity);

    setAlert("humidityHigh", alertState.humidityHigh,
        valid && sensors.humidity > systemState.maxHumidity);
}

void AlertManager::updateWaterTemperatureAlert()
{
    // A missing reading is not an out-of-range reading: the old form compared
    // NaN directly, which silently evaluated false and reported "in range" for
    // a dead sensor. Peltier control is unaffected - updateCooling() reads
    // highWaterTemp / coolerOffTemp directly.
    const bool valid = isfinite(sensors.waterTemp);

    setAlert(
        "waterTempOutOfRange",
        alertState.waterTempOutOfRange,
        valid && sensors.waterTemp > systemState.maxWaterTemp);

    setAlert(
        "waterTempLow",
        alertState.waterTempLow,
        valid && sensors.waterTemp < systemState.minWaterTemp);
}



void AlertManager::updatePHAlert()
{
    const bool valid = isfinite(sensors.ph) && sensors.ph >= 0.0f && sensors.ph <= 14.0f;
    const bool low = valid && sensors.ph < systemState.minPH;
    const bool high = valid && sensors.ph > systemState.maxPH;

    setAlert("phLow", alertState.phLow, low);
    setAlert("phHigh", alertState.phHigh, high);
    setAlert("phOutOfRange", alertState.phOutOfRange, low || high);
}

void AlertManager::updateECAlert()
{
    setAlert(
        "ecLow",
        alertState.ecLow,
        isfinite(sensors.ec) && sensors.ec < systemState.minEC);

    setAlert(
        "ecHigh",
        alertState.ecHigh,
        isfinite(sensors.ec) && sensors.ec > systemState.maxEC);
}

void AlertManager::updateSensorFaultAlert()
{
    const bool sensorFault =

        !isfinite(sensors.temperature) ||

        !isfinite(sensors.humidity) ||

        !isfinite(sensors.waterTemp) ||

        !isfinite(sensors.ph) ||

        !isfinite(sensors.ec) ||

        !isfinite(sensors.waterLevel) ||

        sensors.temperature < -40.0f ||
        sensors.temperature > 100.0f ||

        sensors.humidity < 0.0f ||
        sensors.humidity > 100.0f ||

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
