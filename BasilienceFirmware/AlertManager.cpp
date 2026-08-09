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
    updateLowWaterAlert();

    updateTemperatureAlert();

    updateWaterTemperatureAlert();

    updatePHAlert();

    updateECAlert();

    updateSensorFaultAlert();
} 

void AlertManager::updateLowWaterAlert()
{
    const bool lowWater =
        isfinite(sensors.waterLevel) &&
        sensors.waterLevel <
        systemState.refillStartLevel;

    setAlert("lowWater", alertState.lowWater, lowWater);
}

void AlertManager::updateTemperatureAlert()
{
    setAlert(
        "highTemperature",
        alertState.highTemperature,
        sensors.temperature > systemState.highAirTemp);
}

void AlertManager::updateWaterTemperatureAlert()
{
    setAlert(
        "waterTempOutOfRange",
        alertState.waterTempOutOfRange,
        sensors.waterTemp > systemState.highWaterTemp);
}



void AlertManager::updatePHAlert()
{
    setAlert(
        "phOutOfRange",
        alertState.phOutOfRange,
        sensors.ph < systemState.minPH || sensors.ph > systemState.maxPH);
}

void AlertManager::updateECAlert()
{
    setAlert(
        "ecLow",
        alertState.ecLow,
        sensors.ec < systemState.minEC);
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

    setAlert("sensorFault", alertState.sensorFault, sensorFault || waterLevelFault);
}
