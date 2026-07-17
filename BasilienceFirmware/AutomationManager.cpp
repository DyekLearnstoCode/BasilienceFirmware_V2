#include "AutomationManager.h"

#include "Config.h"
#include "Globals.h"

void AutomationManager::begin()
{
    startupPhase = STARTUP_FOG_ON;

    fogCycleOn = true;

    systemState.currentMode =
        SENSOR_STABILIZATION;

    systemState.stateStartTime =
        millis();
}

void AutomationManager::validateSystem()
{
    updateAlerts();

    if (sensors.waterLevel <
        REFILL_START_LEVEL)
    {
        changeState(REFILLING);

        return;
    }

    changeState(STARTUP);
}

void AutomationManager::update()
{
    if (systemState.manualMode)
        return;

    switch (systemState.currentMode)
    {
        case SENSOR_STABILIZATION:
            handleSensorStabilization();
            break;

        case STARTUP:
            handleStartup();
            break;

        case NORMAL:
            handleNormal();
            break;

         case REFILLING:
    handleRefilling();
    break;   

        default:
            break;
    }
}

void AutomationManager::updateAlerts()
{
    alertState.lowWater =
        sensors.waterLevel <
        REFILL_START_LEVEL;

    alertState.ecLow =
        sensors.ec <
        MIN_EC;

    alertState.phOutOfRange =
        sensors.ph < MIN_PH ||
        sensors.ph > MAX_PH;

    alertState.waterTempOutOfRange =
        sensors.waterTemp <
        LOW_WATER_TEMP ||
        sensors.waterTemp >
        HIGH_WATER_TEMP;

    alertState.highTemperature =
        sensors.temperature >
        HIGH_AIR_TEMP;
}

void AutomationManager::changeState(SystemMode newMode)
{
    SystemMode oldMode =
        systemState.currentMode;

    Serial.println();
    Serial.println("================================");
    Serial.println("STATE CHANGE");
    Serial.println("================================");

    Serial.print("FROM : ");
    Serial.println(getStateName(oldMode));

    Serial.print("TO   : ");
    Serial.println(getStateName(newMode));

    Serial.println("================================");
    Serial.println();

    systemState.currentMode =
        newMode;

    systemState.stateStartTime =
        millis();
}

void AutomationManager::handleSensorStabilization()
{
    actuatorManager.turnOff(FOGGER);

    actuatorManager.turnOff(BLOWER);

    if (millis() -
        systemState.stateStartTime >=
        SENSOR_STABILIZATION_TIME)
    {
        validateSystem();
    }
}

void AutomationManager::handleStartup()
{
    switch (startupPhase)
    {
        case STARTUP_FOG_ON:
        {
            actuatorManager.turnOn(FOGGER);

            actuatorManager.turnOn(BLOWER);

            if (millis() -
                systemState.stateStartTime >=
                STARTUP_ON_TIME)
            {
                actuatorManager.turnOff(FOGGER);

                actuatorManager.turnOff(BLOWER);

                startupPhase =
                    STARTUP_FOG_OFF;

                systemState.stateStartTime =
                    millis();
            }

            break;
        }

        case STARTUP_FOG_OFF:
        {
            actuatorManager.turnOff(FOGGER);

            actuatorManager.turnOff(BLOWER);

            if (millis() -
                systemState.stateStartTime >=
                STARTUP_OFF_TIME)
            {
                fogCycleOn = true;

                changeState(NORMAL);
            }

            break;
        }
    }
}

void AutomationManager::handleNormal()
{
    unsigned long elapsed =
        millis() -
        systemState.stateStartTime;

    unsigned long fogOnTime =
        NORMAL_FOG_ON_TIME;

    unsigned long fogOffTime =
        NORMAL_FOG_OFF_TIME;

    // ====================================
    // Adaptive Fogging
    // ====================================

    if (sensors.temperature > 30.0f)
    {
        fogOnTime = HOT_FOG_ON_TIME;

        fogOffTime = HOT_FOG_OFF_TIME;
    }
    else if (sensors.temperature < 20.0f)
    {
        fogOnTime = COLD_FOG_ON_TIME;

        fogOffTime = COLD_FOG_OFF_TIME;
    }

    // ====================================
    // Fog Cycle
    // ====================================

    if (fogCycleOn)
    {
        actuatorManager.turnOn(FOGGER);

        actuatorManager.turnOn(BLOWER);

        if (elapsed >= fogOnTime)
        {
            fogCycleOn = false;

            systemState.stateStartTime =
                millis();
        }
    }
    else
    {
        actuatorManager.turnOff(FOGGER);

        actuatorManager.turnOff(BLOWER);

        if (elapsed >= fogOffTime)
        {
            fogCycleOn = true;

            systemState.stateStartTime =
                millis();
        }
    }
}

void AutomationManager::handleRefilling()
{
    actuatorManager.turnOn(SOLENOID);

    if (sensors.waterLevel >=
        REFILL_STOP_LEVEL)
    {
        actuatorManager.turnOff(
            SOLENOID);

        changeState(STARTUP);
    }
}

const char* AutomationManager::getStateName(SystemMode mode)
{
    switch (mode)
    {
        case SENSOR_STABILIZATION:
            return "SENSOR_STABILIZATION";

        case STARTUP:
            return "STARTUP";

        case NORMAL:
            return "NORMAL";

        case REFILLING:
            return "REFILLING";

        case DOSING_PH:
            return "DOSING_PH";

        case STABILIZING_PH:
            return "STABILIZING_PH";

        case DOSING_EC:
            return "DOSING_EC";

        case STABILIZING_EC:
            return "STABILIZING_EC";

        case SAFETY_LOCK:
            return "SAFETY_LOCK";

        default:
            return "UNKNOWN";
    }
}