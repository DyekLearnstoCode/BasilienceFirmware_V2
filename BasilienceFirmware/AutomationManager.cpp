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


    if (alertState.lowWater)
    {
        changeState(REFILLING);
        return;
    }


    changeState(STARTUP);
}

void AutomationManager::update()
{
    if(systemState.operationRequest.state ==
       RequestState::ACCEPTED)
    {
        systemState.operationRequest.state =
        RequestState::RUNNING;

        systemState.operationRequest.startedTimestamp =
            millis();

        systemState.operationRequest.lastUpdatedTimestamp =
            systemState.operationRequest.startedTimestamp;
        
        return;
    }

    if(systemState.operationRequest.state ==
       RequestState::RUNNING)
    {
        OperationRequest& request =
            systemState.operationRequest;

        switch(request.operation)
        {
            case OperationType::REFILL:
                if(request.action ==
                   OperationAction::START)
                {
                    changeState(
                        REFILLING);
                }
                break;

            case OperationType::PH_UP:
                if(request.action ==
                   OperationAction::START)
                {
                    systemState.phDirection =
                        PH_UP;

                    changeState(
                        DOSING_PH);
                }
                break;

            case OperationType::PH_DOWN:
                if(request.action ==
                   OperationAction::START)
                {
                    systemState.phDirection =
                        PH_DOWN;

                    changeState(
                        DOSING_PH);
                }
                break;

            case OperationType::GROW_PUMP:
            case OperationType::BLOOM_PUMP:
                if(request.action ==
                   OperationAction::START)
                {
                    changeState(
                        DOSING_EC);
                }
                break;

            case OperationType::RESET_SAFETY:
                if(request.action ==
                   OperationAction::START &&
                   systemState.currentMode ==
                   SAFETY_LOCK)
                {
                    changeState(
                        STARTUP);
                }
                break;

            default:
                break;
        }

        return;
    }

    if(systemState.syncRTC)
    {
        systemState.syncRTC =
            false;

        syncRTCFromFirebase();

        return;
    }

    if (systemState.manualMode)
        return;

    if(systemState.resetSafetyLock)
    {
        systemState.resetSafetyLock =
            false;

        if(systemState.currentMode ==
        SAFETY_LOCK)
        {
            Serial.println(
                "SAFETY LOCK RESET");

            changeState(
                STARTUP);

            return;
        }
    }

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

        case DOSING_PH:
            handleDosingPH();
            break;

        case STABILIZING_PH:
            handleStabilizingPH();
            break;

        case DOSING_EC:
            handleDosingEC();
            break;

        case STABILIZING_EC:
            handleStabilizingEC();
            break;

        case SAFETY_LOCK:
            handleSafetyLock();
            break;

        default:
            break;
    }
}

void AutomationManager::completeCurrentOperation()
{
    if (systemState.operationRequest.state !=
        RequestState::RUNNING)
    {
        return;
    }

    systemState.operationRequest.state =
        RequestState::COMPLETED;

    systemState.operationRequest.completedTimestamp =
        millis();

    systemState.operationRequest.lastUpdatedTimestamp =
        systemState.operationRequest.completedTimestamp;
}

void AutomationManager::updateAlerts()
{
    alertState.lowWater =
        sensors.waterLevel <
        REFILL_START_LEVEL;

    alertState.ecLow =
    sensors.ec <
    systemState.minEC;

    alertState.phOutOfRange =
    sensors.ph < systemState.minPH ||
    sensors.ph > systemState.maxPH;

    alertState.waterTempOutOfRange =
        sensors.waterTemp <
        COOLER_OFF_TEMP ||
        sensors.waterTemp >
        HIGH_WATER_TEMP;

    alertState.highTemperature =
        sensors.temperature >
        HIGH_AIR_TEMP;

    alertState.sensorFault =
    sensors.temperature == 0 ||
    sensors.humidity == 0 ||
    sensors.waterTemp == 0;
}

void AutomationManager::syncRTCFromFirebase()
{
    Serial.println(
        "RTC SYNC REQUESTED");
}

void AutomationManager::changeState(SystemMode newMode)
{
    SystemMode oldMode =
        systemState.currentMode;

    Serial.println();
    Serial.println("================================");
    Serial.println("STATE CHANGE");
    Serial.println("================================");

    // ====================================
    // Dosing Information
    // ====================================

    if(newMode == DOSING_PH)
    {
        if(systemState.phDirection == PH_UP)
        {
            Serial.println(
                "PH UP CORRECTION");

                    Serial.print(
            "Dose Time : ");

        Serial.print(
            systemState.phDoseTime / 1000);

        Serial.println(
            " sec");
        }
        else
        {
            Serial.println(
                "PH DOWN CORRECTION");
        }

        if(newMode == DOSING_EC)
        {
            Serial.println(
                "EC CORRECTION");

            Serial.print(
                "Dose Time : ");

            Serial.print(
                systemState.ecDoseTime / 1000);

            Serial.println(
                " sec");
        }
    }

    Serial.print("FROM : ");
    Serial.println(getStateName(oldMode));

    Serial.print("TO   : ");
    Serial.println(getStateName(newMode));

    if(newMode == SAFETY_LOCK)
    {
        Serial.println(
            "!!! SAFETY LOCK ACTIVATED !!!");
    }

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
    if(systemState.reservoirLocked)
    return;

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

void AutomationManager::updateGrowLightSchedule()
{
    uint8_t currentHour =
        rtcManager.getHour();

    uint8_t currentMinute =
        rtcManager.getMinute();

    int currentTime =
        currentHour * 60 +
        currentMinute;

    int lightOnTime =
        systemState.lightOnHour * 60 +
        systemState.lightOnMinute;

    int lightOffTime =
        systemState.lightOffHour * 60 +
        systemState.lightOffMinute;

    bool lightShouldBeOn = false;

    if(lightOnTime < lightOffTime)
    {
        lightShouldBeOn =
            currentTime >= lightOnTime &&
            currentTime < lightOffTime;
    }
    else
    {
        lightShouldBeOn =
            currentTime >= lightOnTime ||
            currentTime < lightOffTime;
    }

    if(lightShouldBeOn)
    {
        actuatorManager.turnOn(
            GROW_LIGHT);
    }
    else
    {
        actuatorManager.turnOff(
            GROW_LIGHT);
    }
}

void AutomationManager::handleNormal()
{
    updateGrowLightSchedule();

    if(systemState.reservoirLocked)
    {
        actuatorManager.turnOff(FOGGER);
        actuatorManager.turnOff(BLOWER);
        return;
    }

    updateAlerts();

    // ====================================
    // Force Refill Command
    // ====================================

    if(systemState.forceRefill)
    {
        systemState.forceRefill = false;

        changeState(
            REFILLING);

        return;
    }

    if(alertState.sensorFault)
    {
        return;
    }

    //grow light scheduling
    int currentTime =
    rtcManager.getHour() * 60 +
    rtcManager.getMinute();

    int lightOnTime =
        systemState.lightOnHour * 60 +
        systemState.lightOnMinute;

    int lightOffTime =
        systemState.lightOffHour * 60 +
        systemState.lightOffMinute;

    if(currentTime >= lightOnTime &&
    currentTime < lightOffTime)
    {
        actuatorManager.turnOn(
            GROW_LIGHT);
    }
    else
    {
        actuatorManager.turnOff(
            GROW_LIGHT);
    }
    
    // Water Temperature Control
    if(sensors.waterTemp >
    HIGH_WATER_TEMP)
    {
        actuatorManager.turnOn(
            PELTIER);
    }

    if(sensors.waterTemp <
    COOLER_OFF_TEMP)
    {
        actuatorManager.turnOff(
            PELTIER);
    }

    //ph control
    if(alertState.phOutOfRange)
    {
        float error;

        if(sensors.ph < MIN_PH)
        {
            systemState.phDirection =
                PH_UP;

            error =
                MIN_PH -
                sensors.ph;
        }
        else
        {
            systemState.phDirection =
                PH_DOWN;

            error =
                sensors.ph -
                MAX_PH;
        }

        if(error < 0.3f)
        {
            systemState.phDoseTime =
                15000UL;
        }
        else if(error < 1.0f)
        {
            systemState.phDoseTime =
                30000UL;
        }
        else
        {
            systemState.phDoseTime =
                60000UL;
        }

        changeState(DOSING_PH);

        return;
    }

    //ec control
    if(alertState.ecLow)
    {
        float error =
            MIN_EC -
            sensors.ec;

        if(error < 0.2f)
        {
            systemState.ecDoseTime =
                15000UL;
        }
        else if(error < 0.5f)
        {
            systemState.ecDoseTime =
                30000UL;
        }
        else
        {
            systemState.ecDoseTime =
                60000UL;
        }

        changeState(DOSING_EC);

        return;
    }

    //fogging control
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
            systemState.stateStartTime = millis();
        }
    }
    else
    {
        actuatorManager.turnOff(FOGGER);
        actuatorManager.turnOff(BLOWER);

        if (elapsed >= fogOffTime)
        {
            fogCycleOn = true;
            systemState.stateStartTime = millis();
        }
    }
}

void AutomationManager::handleDosingPH()
{
    systemState.reservoirLocked = true;

    if(systemState.phDirection == PH_UP)
    {
        actuatorManager.turnOn(PH_UP_PUMP);

    }
    else if(systemState.phDirection == PH_DOWN)
    {
        actuatorManager.turnOn(PH_DOWN_PUMP);
    }

    if(millis() -
       systemState.stateStartTime >=
       systemState.phDoseTime)
    {
        actuatorManager.turnOff(
            PH_UP_PUMP);

        actuatorManager.turnOff(
            PH_DOWN_PUMP);

        changeState(
            STABILIZING_PH);
    }
}

void AutomationManager::handleStabilizingPH()
{
    actuatorManager.turnOff(
        PH_UP_PUMP);

    actuatorManager.turnOff(
        PH_DOWN_PUMP);

    if(millis() -
       systemState.stateStartTime >=
       PH_STABILIZATION_TIME)
    {
        updateAlerts();

        if(alertState.phOutOfRange)
    {
        systemState.phAttempts++;

        if(systemState.phAttempts >=
        MAX_PH_ATTEMPTS)
        {
            changeState(
                SAFETY_LOCK);

            return;
        }

        changeState(
            DOSING_PH);

        return;
    }
        systemState.phAttempts = 0;
        systemState.phDirection = PH_NONE;

        systemState.reservoirLocked = false;

        completeCurrentOperation();

        changeState(NORMAL);
    }
}

void AutomationManager::handleDosingEC()
{
      systemState.reservoirLocked = true;

    actuatorManager.turnOn(GROW_PUMP);

    actuatorManager.turnOn(BLOOM_PUMP);

    if(millis() -
       systemState.stateStartTime >=
       systemState.ecDoseTime)
    {
        actuatorManager.turnOff(
            GROW_PUMP);

        actuatorManager.turnOff(
            BLOOM_PUMP);

        changeState(
            STABILIZING_EC);
    }
}

void AutomationManager::handleStabilizingEC()
{
   actuatorManager.turnOff(
        GROW_PUMP);

    actuatorManager.turnOff(
        BLOOM_PUMP);

    if(millis() -
       systemState.stateStartTime >=
       EC_STABILIZATION_TIME)
    {
        updateAlerts();

        if(alertState.ecLow)
        {
            systemState.ecAttempts++;

            if(systemState.ecAttempts >=
               MAX_EC_ATTEMPTS)
            {
                changeState(
                    SAFETY_LOCK);

                return;
            }

            changeState(
                DOSING_EC);

            return;
        }

        systemState.ecAttempts = 0;

        systemState.reservoirLocked = false;

        completeCurrentOperation();

        changeState(NORMAL);
    }
}

void AutomationManager::handleRefilling()
{
    systemState.reservoirLocked = true;

    actuatorManager.turnOn(SOLENOID);

    if (sensors.waterLevel >=
        REFILL_STOP_LEVEL)
    {
        actuatorManager.turnOff(SOLENOID);

        systemState.reservoirLocked = false;
        completeCurrentOperation();
        changeState(STARTUP);
    }
}

void AutomationManager::handleSafetyLock()
{
    actuatorManager.turnOffAll();

    systemState.reservoirLocked =
        true;
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
