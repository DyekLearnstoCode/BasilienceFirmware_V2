#include "AlertManager.h"

#include "Globals.h"
void AlertManager::begin()
{
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
    alertState.lowWater =
        sensors.waterLevel <
        systemState.refillStartLevel;
}

void AlertManager::updateTemperatureAlert()
{
    alertState.highTemperature =
        sensors.temperature >
        systemState.highAirTemp;
}

void AlertManager::updateWaterTemperatureAlert()
{
    alertState.waterTempOutOfRange =
        sensors.waterTemp >
        systemState.highWaterTemp;
}



void AlertManager::updatePHAlert()
{
    alertState.phOutOfRange =
        sensors.ph <
        systemState.minPH ||

        sensors.ph >
        systemState.maxPH;
}

void AlertManager::updateECAlert()
{
    alertState.ecLow =
        sensors.ec <
        systemState.minEC;
}

void AlertManager::updateSensorFaultAlert()
{
    alertState.sensorFault =

        isnan(sensors.temperature) ||

        isnan(sensors.humidity) ||

        isnan(sensors.waterTemp) ||

        isnan(sensors.ph) ||

        isnan(sensors.ec) ||

        sensors.temperature < -40.0f ||
        sensors.temperature > 100.0f ||

        sensors.humidity < 0.0f ||
        sensors.humidity > 100.0f ||

        sensors.waterTemp < 0.0f ||
        sensors.waterTemp > 100.0f ||

        sensors.ph < 0.0f ||
        sensors.ph > 14.0f ||

        sensors.ec < 0.0f;
}