#include "AutomationManager.h"

#include "Config.h"
#include "Globals.h"

#include "ActuatorManager.h"

extern ActuatorManager actuatorManager;


void AutomationManager::begin()
{
    startupPhase = FOGGING_PHASE;

    systemState.currentMode = STARTUP;

    systemState.stateStartTime = millis();
}

void AutomationManager::update()
{
    if (systemState.manualMode)
        return;

    switch (systemState.currentMode)
{
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

void AutomationManager::handleStartup()
{
    switch (startupPhase)
    {
        case FOGGING_PHASE:
        {
            actuatorManager.turnOn(FOGGER);

            actuatorManager.turnOn(BLOWER);

            if (millis() - systemState.stateStartTime >=
                STARTUP_FOGGING_TIME)
            {
                actuatorManager.turnOff(FOGGER);

                startupPhase = REST_PHASE;

                systemState.stateStartTime = millis();
            }

            break;
        }

        case REST_PHASE:
        {
            actuatorManager.turnOff(FOGGER);

            actuatorManager.turnOn(BLOWER);

            if (millis() - systemState.stateStartTime >=
                STARTUP_REST_TIME)
            {
                changeState(NORMAL);
            }

            break;
        }
    }
}

void AutomationManager::handleNormal()
{
    actuatorManager.turnOn(BLOWER);

    if (sensors.waterLevel < 20.0f)
    {
        changeState(REFILLING);
    }
}

void AutomationManager::handleRefilling()
{
    actuatorManager.turnOn(SOLENOID);

    if (sensors.waterLevel >= 80.0f)
    {
        actuatorManager.turnOff(SOLENOID);

        changeState(NORMAL);
    }
}

const char* AutomationManager::getStateName(SystemMode mode)
{
    switch (mode)
    {
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